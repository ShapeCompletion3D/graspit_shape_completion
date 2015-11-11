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

//#include "graspit_shape_completion/ros_graspit_interface.h"
#include "include/graspit_shape_completion/ros_graspit_interface.h"
//#include "ros_graspit_interface.h"

#include <boost/foreach.hpp>
#include <cmath>
#include <Inventor/actions/SoGetBoundingBoxAction.h>

#include <src/DBase/DBPlanner/ros_database_manager.h>
#include <src/DBase/graspit_db_model.h>
#include <src/Collision/collisionStructures.h>
//#include <include/EGPlanner/searchEnergy.h>
#include <include/mytools.h>
#include <include/world.h>
#include <include/body.h>
#include <include/graspitGUI.h>
#include <include/ivmgr.h>
#include <include/scanSimulator.h>
#include <include/pr2Gripper.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <include/EGPlanner/egPlanner.h>
#include <include/EGPlanner/simAnnPlanner.h>
//#include <include/EGPlanner/searchState.h>
#include <include/grasp.h>
#include <include/triangle.h>
#include <geometry_msgs/Point.h>

#include <QtGui>
//------------------------- Convenience functions -------------------------------

//namespace std
//{
//    template<> struct less<position>
//    {
//       bool operator() (const position& lhs, const position& rhs) const
//       {
//           if (lhs.x() != rhs.x())
//           {
//               return lhs.x() < rhs.x();
//           }
//           else if (lhs.y() != rhs.y())
//           {
//               return lhs.y() < rhs.y();
//           }
//           else
//           {
//               return lhs.z() < rhs.z();
//           }
//       }
//    };
//}

namespace graspit_ros_planning
{

void meshMsgToVerticesTriangles(const shape_msgs::Mesh &mesh, std::vector<position> *vertices, std::vector<int> *triangles)
{
    for(int i=0 ; i < mesh.vertices.size();i++)
    {
        geometry_msgs::Point p = mesh.vertices.at(i);
        vertices->push_back(position(p.x,p.y,p.z));
    }

    for(int i=0 ; i < mesh.triangles.size();i++)
    {
        shape_msgs::MeshTriangle t = mesh.triangles.at(i);

//        geometry_msgs::Point p0 = mesh.vertices.at(t.vertex_indices.at(0));
//        geometry_msgs::Point p1 = mesh.vertices.at(t.vertex_indices.at(1));
//        geometry_msgs::Point p2 = mesh.vertices.at(t.vertex_indices.at(2));

//        triangles.push_back(Triangle(position(p0.x, p0.y,p0.z),
//                                     position(p1.x, p1.y,p1.z),
//                                     position(p2.x, p2.y,p2.z)));
        triangles->push_back(t.vertex_indices.at(0));
        triangles->push_back(t.vertex_indices.at(1));
        triangles->push_back(t.vertex_indices.at(2));
    }
}

void verticesToMeshMsg(shape_msgs::Mesh &mesh, std::vector<position> *vertices)
{
    //std::map<position, int> point_to_indice;

    for(int i=0 ; i < vertices->size();i++)
    {
        position p = vertices->at(i);
        geometry_msgs::Point gp;
        gp.x = p.x();
        gp.y = p.y();
        gp.z = p.z();
        mesh.vertices.push_back(gp);

        //point_to_indice[p] = i;
    }

//    for(int i=0 ; i < triangles->size();i++)
//    {
//        Triangle t = triangles.at(i);
//        int v1 = point_to_indice.find(t.v1);
//        int v2 = point_to_indice.find(t.v2);
//        int v3 = point_to_indice.find(t.v3);
//        mesh.triangles.push_back(shape_msgs::MeshTriangle(v1,v2,v3));
//    }
}


RosGraspitInterface::RosGraspitInterface() :
  root_nh_(NULL), priv_nh_(NULL),
    get_segmented_meshed_scene_client("get_segmented_meshed_scene", true),
    complete_mesh_client("complete_mesh", true)
{
}

RosGraspitInterface::~RosGraspitInterface()
{
  ROS_INFO("ROS GraspIt node stopping");
  ros::shutdown();
  delete root_nh_;
  delete priv_nh_;
}

//------------------------- Main class  -------------------------------

int RosGraspitInterface::init(int argc, char **argv)
{
  //copy the arguments somewhere else so we can pass them to ROS
  int ros_argc = argc;
  char** ros_argv = new char*[argc];
  for (int i = 0; i < argc; i++)
  {
    ros_argv[i] = new char[strlen(argv[i])];
    strcpy(ros_argv[i], argv[i]);
  }
  //see if a node name was requested
  std::string node_name("ros_graspit_interface");
  for (int i = 0; i < argc - 1; i++)
  {
    //std::cerr << argv[i] << "\n";
    if (!strcmp(argv[i], "_name"))
    {
      node_name = argv[i + 1];
    }
  }
  //init ros
  ros::init(ros_argc, ros_argv, node_name.c_str());
  //ROS_INFO("Using node name %s", node_name.c_str());
  //clean up ros arguments
  for (int i = 0; i < argc; i++)
  {
    delete ros_argv[i];
  }
  delete ros_argv;
  //init node handles
  root_nh_ = new ros::NodeHandle("");
  priv_nh_ = new ros::NodeHandle("~");

  srand(1);
  ROS_INFO("MAKING SHAPE COMPLETION UI");

  QPushButton * captureSceneButton = new QPushButton("Capture Scene");
  QPushButton * shapeCompletionButton = new QPushButton("Complete Selected Mesh");

  captureSceneButton->setDefault(true);
  shapeCompletionButton->setDefault(true);

  QDialogButtonBox *shapeCompletionControlBox = new QDialogButtonBox(Qt::Vertical);

  shapeCompletionControlBox->addButton(captureSceneButton, QDialogButtonBox::ActionRole);
  shapeCompletionControlBox->addButton(shapeCompletionButton, QDialogButtonBox::ActionRole);
  shapeCompletionControlBox->resize(QSize(200,100));
  shapeCompletionControlBox->show();

  QObject::connect(captureSceneButton, SIGNAL(clicked()), this, SLOT(onCaptureSceneButtonPressed()));
  QObject::connect(shapeCompletionButton, SIGNAL(clicked()), this, SLOT(onCompleteShapeButtonPressed()));


  ROS_INFO("ROS GraspIt node ready");
  return 0;
}

int RosGraspitInterface::mainLoop()
{
  ros::spinOnce();
  return 0;
}

void RosGraspitInterface::onCaptureSceneButtonPressed()
{
    ROS_INFO("onCaptureSceneButtonPressed\n");
    graspit_shape_completion::GetSegmentedMeshedSceneGoal goal;

    get_segmented_meshed_scene_client.sendGoal(goal,  boost::bind(&RosGraspitInterface::receivedMeshedSceneCB, this, _1, _2),
                actionlib::SimpleActionClient<graspit_shape_completion::GetSegmentedMeshedSceneAction>::SimpleActiveCallback(),
                actionlib::SimpleActionClient<graspit_shape_completion::GetSegmentedMeshedSceneAction>::SimpleFeedbackCallback());
}


void RosGraspitInterface::receivedMeshedSceneCB(const actionlib::SimpleClientGoalState& state,
                       const graspit_shape_completion::GetSegmentedMeshedSceneResultConstPtr& result)
{
    ROS_INFO("Sucessfully recieved meshed scene");

    std::for_each(result->meshes.begin(),
                  result->meshes.end(),
                  boost::bind(&RosGraspitInterface::addMesh, this, _1));
}


void RosGraspitInterface::addMesh(shape_msgs::Mesh mesh)
{
    std::vector<int> triangles;
    std::vector<position> vertices;
    meshMsgToVerticesTriangles(mesh, &vertices, &triangles);

    Body *b = new Body(graspItGUI->getMainWorld());
    b->loadGeometryMemory(vertices, triangles);
}


void RosGraspitInterface::onCompleteShapeButtonPressed()
{
    ROS_INFO("onCompleteShapeButtonPressed\n");
    graspit_shape_completion::CompleteMeshGoal goal;

    Body *b = graspItGUI->getMainWorld()->getSelectedBody(0);

    std::vector<position> vertices;

    b->getGeometryVertices(&vertices);

    verticesToMeshMsg(goal.partial_mesh, &vertices);

    complete_mesh_client.sendGoal(goal,  boost::bind(&RosGraspitInterface::completeMeshCB, this, _1, _2),
                actionlib::SimpleActionClient<graspit_shape_completion::CompleteMeshAction>::SimpleActiveCallback(),
                actionlib::SimpleActionClient<graspit_shape_completion::CompleteMeshAction>::SimpleFeedbackCallback());
}



void RosGraspitInterface::completeMeshCB(const actionlib::SimpleClientGoalState& state,
                       const graspit_shape_completion::CompleteMeshResultConstPtr& result)
{
    ROS_INFO("Sucessfully recieved completed mesh");
    std::vector<int> triangles;
    std::vector<position> vertices;
    meshMsgToVerticesTriangles(result->completed_mesh, &vertices, &triangles);

    Body *b = graspItGUI->getMainWorld()->getSelectedBody(0);
    b->loadGeometryMemory(vertices, triangles);
}


}
