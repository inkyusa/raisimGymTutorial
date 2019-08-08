//
// Created by Jemin on 3/27/19.
// Modified by Inkyu on 22/July/2019 (based on Laikago Environment)
// MIT License
//
// Copyright (c) 2019-2019 Robotic Systems Lab, ETH Zurich
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

/* Convention
*   action space = force (continuous)                        n = 1, index =0
*
*   observation space = [ x (cart position)                  n =  1, index=0
                          theta (tilted angle),              n =  1, index=1
                          x_dot (cart linear velocity),      n =  1, index=2
*                         theta_dot (tilted angle velocity,  n =  1, index=3] total 4
*/


#include <stdlib.h>
#include <cstdint>
#include <set>
#include <raisim/OgreVis.hpp>
#include "RaisimGymEnv.hpp"
#include "visSetupCallback.hpp"

#include "visualizer/raisimKeyboardCallback.hpp"
#include "visualizer/helper.hpp"
#include "visualizer/guiState.hpp"
#include "visualizer/raisimBasicImguiPanel.hpp"

using namespace std;
#define deg2rad(ang) ((ang) * M_PI / 180.0)
#define rad2deg(ang) ((ang) * 180.0 / M_PI)

namespace raisim {

class ENVIRONMENT : public RaisimGymEnv {

 public:

  explicit ENVIRONMENT(const std::string& resourceDir, const YAML::Node& cfg, bool visualizable) :
      RaisimGymEnv(resourceDir, cfg), distribution_(0.0, 0.2), visualizable_(visualizable) {

    /// add objects
    cout<<resourceDir<<endl;
    cartpole_ = world_->addArticulatedSystem(resourceDir+"/cartpole.urdf");
    cartpole_->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
    auto ground = world_->addGround();
    world_->setERP(0,0);
    /// get robot data
    gcDim_ = cartpole_->getGeneralizedCoordinateDim(); //will be two; cart position and pole angle
    gvDim_ = cartpole_->getDOF(); // will be two; cart linear velocity and pole angular velocity.
    nJoints_ = 2;
    /// initialize containers
    gc_.setZero(gcDim_); gc_init_.setZero(gcDim_);
    gv_.setZero(gvDim_); gv_init_.setZero(gvDim_);
    cartpole_->setGeneralizedForce(Eigen::VectorXd::Zero(nJoints_));

    /// MUST BE DONE FOR ALL ENVIRONMENTS
    obDim_ = 4; /// x,theta, x_dot, theta_dot
    actionDim_ = 1;
    actionMean_.setZero(actionDim_); actionStd_.setZero(actionDim_);
    obMean_.setZero(obDim_); obStd_.setZero(obDim_);

    /// action & observation scaling
    actionMean_ = gc_init_.tail(actionDim_);
    actionStd_.setConstant(0.6);

    obMean_.setZero();
    obStd_ << 3, //cart position (meter)
              2*M_PI, //pole angle (rad)
              0.1, //cart velocity (m/s)
              10; //pole angular velocity (rad/sec)
    
    //Reward coefficients
    forceRewardCoeff_ = cfg["forceRewardCoeff"].as<double>();
    gui::rewardLogger.init({"reward", "forceReward"});
    reward_=0;

    /// visualize if it is the first environment
    if (visualizable_) {
      auto vis = raisim::OgreVis::get();

      /// these method must be called before initApp
      vis->setWorld(world_.get());
      vis->setWindowSize(1800, 1200);
      vis->setImguiSetupCallback(imguiSetupCallback);
      vis->setImguiRenderCallback(imguiRenderCallBack);
      vis->setKeyboardCallback(raisimKeyboardCallback);
      vis->setSetUpCallback(setupCallback);
      vis->setAntiAliasing(2);

      /// starts visualizer thread
      vis->initApp();
      cartpoleVisual_ = vis->createGraphicalObject(cartpole_, "Cartpole");
      vis->createGraphicalObject(ground, 20, "floor", "checkerboard_green");
      desired_fps_ = 50.;
      vis->setDesiredFPS(desired_fps_);
      vis->select(cartpoleVisual_->at(0), false);
      vis->getCameraMan()->setYawPitchDist(Ogre::Radian(-1.0), Ogre::Radian(-1.0), 3);
    }
  }

  ~ENVIRONMENT() final = default;

  void init() final { }

  void reset() final {
    cartpole_->setState(gc_init_, gv_init_);
    updateObservation();
    if(visualizable_)
      gui::rewardLogger.clean();
  }

  float step(const Eigen::Ref<EigenVec>& action) final {
    /// action scaling
    actionScaled_ = action.cast<double>()*1000;
    Eigen::Vector2d gf; gf.setZero();
    gf.head(1) = actionScaled_; //set only the first element with action (i.e., cart pushing force)
    cartpole_->setGeneralizedForce(gf);

    auto loopCount = int(control_dt_ / simulation_dt_ + 1e-10);
    auto visDecimation = int(1. / (desired_fps_ * simulation_dt_) + 1e-10);

    for(int i=0; i<loopCount; i++) {
      world_->integrate();

      if (visualizable_ && visualizeThisStep_ && visualizationCounter_ % visDecimation == 0)
        raisim::OgreVis::get()->renderOneFrame();

      visualizationCounter_++;
    }
    updateObservation();
    forceReward_ = forceRewardCoeff_ * cartpole_->getGeneralizedForce().squaredNorm();

    reward_=1.0;
    if(visualizeThisStep_) {
        gui::rewardLogger.log("forceReward", forceReward_);
        gui::rewardLogger.log("reward", reward_);
    }
   return forceReward_ + reward_;
  }

  void updateExtraInfo() final {
    extraInfo_["reward"] = reward_;
  }

  void updateObservation() {
    cartpole_->getState(gc_, gv_);
    obDouble_.setZero(obDim_); obScaled_.setZero(obDim_);
    obDouble_ << gc_,gv_; //x, theta, x_dot, theta_dot
    obScaled_ = (obDouble_-obMean_).cwiseQuotient(obStd_);
  }

  void observe(Eigen::Ref<EigenVec> ob) final {
    /// convert it to float
    ob = obScaled_.cast<float>();
  }

  bool isTerminalState(float& terminalReward) final {
    terminalReward = float(terminalRewardCoeff_);
    //If the angle of pole is greater than +-50 degs or the cart position is greater than +-2m,
    //treat them as terminal conditions
    if(rad2deg(abs(obDouble_[1]))> 50. || abs(obDouble_[0])>2.0) return true;
    terminalReward = 0.f;
    return false;
  }

  void setSeed(int seed) final {
    std::srand(seed);
  }

  void close() final {
  }

 private:
  int gcDim_, gvDim_, nJoints_;
  double reward_;
  bool visualizable_ = false;
  std::normal_distribution<double> distribution_;
  raisim::ArticulatedSystem* cartpole_;
  std::vector<GraphicObject> * cartpoleVisual_;
  Eigen::VectorXd gc_init_, gv_init_, gc_, gv_, actionScaled_,torque_;
  double terminalRewardCoeff_ = -10.;
  double forceRewardCoeff_ = 0., forceReward_ = 0.;
  double desired_fps_ = 60.;
  int visualizationCounter_=0;
  Eigen::VectorXd actionMean_, actionStd_, obMean_, obStd_;
  Eigen::VectorXd obDouble_, obScaled_;
};

}

