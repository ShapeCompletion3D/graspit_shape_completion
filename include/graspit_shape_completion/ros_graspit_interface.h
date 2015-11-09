/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef _ROS_GRASPIT_INTERFACE_H_
#define _ROS_GRASPIT_INTERFACE_H_

#include <map>
#include <QObject>

//GraspIt! includes
#include <include/plugin.h>

class GraspitDBModel;
class Pr2Gripper2010;
class Body;
class transf;
class GraspableBody;

#include <ros/ros.h>

#include <manipulation_msgs/Grasp.h>
#include <manipulation_msgs/GraspPlanning.h>

#include "graspit_shape_completion/GetSegmentedMeshedSceneAction.h"
#include "graspit_shape_completion/CompleteMeshAction.h"
#include <actionlib/client/simple_action_client.h>

namespace graspit_ros_planning
{

//! Main class, combining a ROS node with a GraspIt! interface
/*! Note that this class inherits from GraspIt's Plugin class and implements the necessary functions to serve
  as a GraspIt plugin. See include/plugin.h in the GraspIt code for the base class.

  Provides a number of ROS services that directly operate on the GraspIt world, such as loading objects or
  obstacles, simulating 3D scans of objects, etc.

  In particular, note that this class uses the mainLoop() function to perform the ROS even management calls.
*/
class RosGraspitInterface : public QObject, public Plugin
{

    Q_OBJECT

private:
  //! Node handle in the root namespace
  ros::NodeHandle *root_nh_;

  //! Node handle in the private namespace
  ros::NodeHandle *priv_nh_;

  actionlib::SimpleActionClient<graspit_shape_completion::GetSegmentedMeshedSceneAction> get_segmented_meshed_scene_client;
  actionlib::SimpleActionClient<graspit_shape_completion::CompleteMeshAction> complete_mesh_client;

  void addMesh(shape_msgs::Mesh mesh);


public:
  //! Inits ROS, but (for now) without passing any arguments
  RosGraspitInterface();
  //! Deletes the node handle and the db manager
  ~RosGraspitInterface();
  //! Creates the node handles, advertises services, connects to the database
  virtual int init(int argc, char **argv);
  //! Simply calls ros::spinOnce() to process the ROS event loop
  virtual int mainLoop();

  void receivedMeshedSceneCB(const actionlib::SimpleClientGoalState& state, const graspit_shape_completion::GetSegmentedMeshedSceneResultConstPtr& result);
  void completeMeshCB(const actionlib::SimpleClientGoalState& state, const graspit_shape_completion::CompleteMeshResultConstPtr& result);

  public slots:

      void onCaptureSceneButtonPressed();
      void onCompleteShapeButtonPressed();

};


}

#endif
