/*
 * Copyright (c) 2020, Dawid Seredynski
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the <organization> nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **/

/*
 * \file  breakable_joint.cpp
 *
 * \brief A breakable joint plugin for Gazebo
 *
 * \author  Dawid Seredynski (dawid.seredynski@pw.edu.pl)
 *
 */


#include <algorithm>
#include <assert.h>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>
#include <boost/bind.hpp>

namespace gazebo {

  class BreakableJoint : public ModelPlugin {

    public:
    BreakableJoint();
      ~BreakableJoint();
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    protected:
      virtual void UpdateChild();
      virtual void FiniChild();

    private:

      physics::WorldPtr world;
      physics::ModelPtr parent;
      event::ConnectionPtr update_connection_;

      std::string joint_name_;
      double pos_min_;
      double pos_max_;

      bool joint_is_broken_;

      physics::JointPtr joint_;

      // ROS STUFF
      ros::NodeHandle* rosnode_;

      std::string robot_namespace_;
  };

  BreakableJoint::BreakableJoint() {}

  // Destructor
  BreakableJoint::~BreakableJoint() {
    delete rosnode_;
  }

  // Load the controller
  void BreakableJoint::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

    this->parent = _parent;
    this->world = _parent->GetWorld();

    this->robot_namespace_ = "";
    if (!_sdf->HasElement("robotNamespace")) {
      ROS_INFO_NAMED("breakable_joint", "BreakableJoint Plugin missing <robotNamespace>, defaults to \"%s\"",
          this->robot_namespace_.c_str());
    } else {
      this->robot_namespace_ =
        _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
    }

    // TODO write error if joint doesn't exist!
    this->joint_name_ = "joint_1";
    if (!_sdf->HasElement("joint_name")) {
      ROS_WARN_NAMED("breakable_joint", "BreakableJoint Plugin (ns = %s) missing <joint_name>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->joint_name_.c_str());
    } else {
      this->joint_name_ = _sdf->GetElement("joint_name")->Get<std::string>();
    }

    this->pos_min_ = -0.01;
        if (!_sdf->HasElement("pos_min")) {
          ROS_WARN_NAMED("breakable_joint", "BreakableJoint Plugin (ns = %s) missing <pos_min>, defaults to %f",
              this->robot_namespace_.c_str(), this->pos_min_);
        } else {
          this->pos_min_ = _sdf->GetElement("pos_min")->Get<double>();
        }

    this->pos_max_ = 0.01;
  	if (!_sdf->HasElement("pos_max")) {
  	  ROS_WARN_NAMED("breakable_joint", "BreakableJoint Plugin (ns = %s) missing <pos_max>, defaults to %f",
  		  this->robot_namespace_.c_str(), this->pos_max_);
  	} else {
  	  this->pos_max_ = _sdf->GetElement("pos_max")->Get<double>();
  	}

    // Initialize flag
    this->joint_is_broken_ = false;

    joint_ = this->parent->GetJoint(this->joint_name_);

    if (!joint_) {
      char error[200];
      snprintf(error, 200,
          "BreakableJoint Plugin (ns = %s) couldn't get left front hinge joint named \"%s\"",
          this->robot_namespace_.c_str(), this->joint_name_.c_str());
      gzthrow(error);
    }

    rosnode_ = new ros::NodeHandle(this->robot_namespace_);
    ROS_INFO_NAMED("breakable_joint", "Starting BreakableJoint Plugin (ns = %s)", this->robot_namespace_.c_str());

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&BreakableJoint::UpdateChild, this));

  }

  // Update the controller
  void BreakableJoint::UpdateChild() {
    if (!this->joint_is_broken_) {
      double pos = this->joint_->Position(0);
      if (pos < this->pos_min_ || pos > this->pos_max_) {      
        this->joint_->SetParam("friction", 0, 0.0); 
        this->joint_is_broken_ = true;
        ROS_INFO_NAMED("breakable_joint", "BreakableJoint joint \"%s\" is broken",
          this->joint_name_.c_str());
      }
    }
  }

  // Finalize the controller
  void BreakableJoint::FiniChild() {
    rosnode_->shutdown();
  }
  GZ_REGISTER_MODEL_PLUGIN(BreakableJoint)
}
