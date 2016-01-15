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

#include "include/graspit_shape_completion/ros_graspit_interface.h"

#include <boost/foreach.hpp>
#include <cmath>
#include <Inventor/actions/SoGetBoundingBoxAction.h>

#include <src/DBase/DBPlanner/ros_database_manager.h>
#include <src/DBase/graspit_db_model.h>
#include <src/Collision/collisionStructures.h>

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


#include <include/grasp.h>
#include <include/triangle.h>
#include <geometry_msgs/Point.h>
#include <QtGui>
#include <thread>


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

        triangles->push_back(t.vertex_indices.at(0));
        triangles->push_back(t.vertex_indices.at(1));
        triangles->push_back(t.vertex_indices.at(2));
    }
}

void verticesToMeshMsg(shape_msgs::Mesh &mesh, std::vector<position> *vertices)
{

    for(int i=0 ; i < vertices->size();i++)
    {
        position p = vertices->at(i);
        geometry_msgs::Point gp;
        gp.x = p.x();
        gp.y = p.y();
        gp.z = p.z();
        mesh.vertices.push_back(gp);

    }

}


RosGraspitInterface::RosGraspitInterface() :
    root_nh_(NULL),
    priv_nh_(NULL),
    get_segmented_meshed_scene_client(NULL),
    complete_mesh_client(NULL)
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
    std::cout << "ThreadId" <<std::this_thread::get_id() << std::endl;
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

  //init node handles
  root_nh_ = new ros::NodeHandle("");
  priv_nh_ = new ros::NodeHandle("~");

  get_segmented_meshed_scene_client= new actionlib::SimpleActionClient<graspit_shape_completion::GetSegmentedMeshedSceneAction>("/get_segmented_meshed_scene", true);
  complete_mesh_client = new actionlib::SimpleActionClient<graspit_shape_completion::CompleteMeshAction>("/complete_mesh", true);

  meshed_scene_repub = priv_nh_->advertise<graspit_shape_completion::GetSegmentedMeshedSceneResult>("/get_segmented_meshed_scene",1);
  meshed_scene_sub = priv_nh_->subscribe("/get_segmented_meshed_scene",
                                                                        10,
                                                                        &RosGraspitInterface::getSegmentedMeshesCB,
                                                                        this);
  ROS_INFO("Using node name %s", node_name.c_str());
  for (int i = 0; i < argc; i++)
  {
    delete ros_argv[i];
  }
  delete ros_argv;


  srand(1);
  ROS_INFO("MAKING SHAPE COMPLETION UI");

  QPushButton * captureSceneButton = new QPushButton("Capture Scene");
  QPushButton * shapeCompletionButton = new QPushButton("Complete Selected Mesh");
  QPushButton * graspPlanningButton = new QPushButton("Plan Grasps");

  captureSceneButton->setDefault(true);
  shapeCompletionButton->setDefault(true);

  QWidget *shapeCompletionControlBox = new QWidget();

  scene_segmentation_time = new QLabel(tr("Scene Segmentation Time:?"));
  target_total_time = new QLabel(tr("Target Completion Time:?"));
  target_preprocess_time = new QLabel(tr("Target Completion Time:?"));
  target_completion_time = new QLabel(tr("Target Completion Time:?"));
  target_postprocess_time = new QLabel(tr("Target Completion Time:?"));
  grasp_planning_time = new QLabel(tr("Grasp Planning Time:?"));

  QGridLayout *mainLayout = new QGridLayout;
  mainLayout->addWidget(scene_segmentation_time, 0, 0);
  mainLayout->addWidget(target_total_time, 1,0);
  mainLayout->addWidget(target_preprocess_time, 2,0);
  mainLayout->addWidget(target_completion_time, 3,0);
  mainLayout->addWidget(target_postprocess_time, 4,0);
  mainLayout->addWidget(grasp_planning_time, 5,0);
  mainLayout->addWidget(captureSceneButton, 0, 1, 1, 2);
  mainLayout->addWidget(shapeCompletionButton, 1, 1, 1, 2);
  mainLayout->addWidget(graspPlanningButton, 2, 1, 1, 2);

  shapeCompletionControlBox->setLayout(mainLayout);


  //shapeCompletionControlBox->resize(QSize(200,100));

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

void RosGraspitInterface::onGraspPlanningButtonPressed()
{
    //graspItGUI->getMainWindow()->eigenGraspPlannerActivated();
}

void RosGraspitInterface::onCaptureSceneButtonPressed()
{
    int numGB = graspItGUI->getMainWorld()->getNumGB();
    for (int i=0; i < numGB; i++)
    {
        graspItGUI->getMainWorld()->removeElementFromSceneGraph(graspItGUI->getMainWorld()->getGB(i));
        graspItGUI->getMainWorld()->destroyElement(graspItGUI->getMainWorld()->getGB(i), true);
    }


    ROS_INFO("onCaptureSceneButtonPressed\n");
    std::cout << "ThreadId" <<std::this_thread::get_id() << std::endl;
    graspit_shape_completion::GetSegmentedMeshedSceneGoal goal;

    get_segmented_meshed_scene_client->sendGoal(goal,  boost::bind(&RosGraspitInterface::receivedMeshedSceneCB, this, _1, _2),
                actionlib::SimpleActionClient<graspit_shape_completion::GetSegmentedMeshedSceneAction>::SimpleActiveCallback(),
                actionlib::SimpleActionClient<graspit_shape_completion::GetSegmentedMeshedSceneAction>::SimpleFeedbackCallback());
}

void RosGraspitInterface::getSegmentedMeshesCB(const graspit_shape_completion::GetSegmentedMeshedSceneResultConstPtr& result)
{
    if(QThread::currentThread() != QApplication::instance()->thread())
    {
        meshed_scene_repub.publish(result);
    }
    else
    {
        for (int i =0; i < result->meshes.size(); i++)
        {
            int new_mesh_index = graspItGUI->getMainWorld()->getNumGB();
            addMesh(new_mesh_index, result->meshes.at(i), result->offsets.at(i));
        }
    }

    ROS_INFO("Sucessfully recieved meshed scene");


}

void RosGraspitInterface::receivedMeshedSceneCB(const actionlib::SimpleClientGoalState& state,
                       const graspit_shape_completion::GetSegmentedMeshedSceneResultConstPtr& result)
{
    getSegmentedMeshesCB(result);
    scene_segmentation_time->setText(QString("Scene Segmentation Time (ms): ") + QString::number(result->completion_time));
    ROS_INFO("Sucessfully recieved meshed scene");
}


void RosGraspitInterface::addMesh(int mesh_index, shape_msgs::Mesh mesh, geometry_msgs::Vector3 offset)
{
    std::vector<int> triangles;
    std::vector<position> vertices;
    meshMsgToVerticesTriangles(mesh, &vertices, &triangles);

    GraspableBody *b = new GraspableBody(graspItGUI->getMainWorld());
    if (b->loadGeometryMemory(vertices, triangles) == SUCCESS)
    {
        ROS_INFO("setting mesh name\n");
        QString meshname = QString("mesh_") +  QString::number(mesh_index);
        b->setName(meshname);

        ROS_INFO("About to add IVMAT\n");
        b->addIVMat(true);

        ROS_INFO("About to add to IVC\n");
        b->addToIvc();

        ROS_INFO("About to add to main world\n");
        graspItGUI->getMainWorld()->addBody(b);

        ROS_INFO("offsetting mesh\n");
        transf *t = new transf(mat3::IDENTITY, vec3(offset.x, offset.y, offset.z));
        b->setTran(*t);

        ROS_INFO("Added Mesh: %s",b->getName().toStdString().c_str());

    }
    else
    {
        ROS_INFO("Failed to load mesh geometry; Mesh: %s",b->getName().toStdString().c_str());
    }


}


void RosGraspitInterface::onCompleteShapeButtonPressed()
{
    ROS_INFO("onCompleteShapeButtonPressed\n");
    graspit_shape_completion::CompleteMeshGoal goal;

    int numSelectedBodies = graspItGUI->getMainWorld()->getNumSelectedBodies();
    if (numSelectedBodies == 0)
    {
        ROS_INFO("No Selected Bodies!\n");
        return;
    }
    selected_body = graspItGUI->getMainWorld()->getSelectedBody(0);

    ROS_INFO("getting SelectedBody\n");

    std::vector<position> vertices;

    ROS_INFO("getting Body Vertices\n");
    selected_body->getGeometryVertices(&vertices);

    ROS_INFO("converting Vertices to Mesh Message\n");
    verticesToMeshMsg(goal.partial_mesh, &vertices);

    ROS_INFO("Sending Goal\n");
    complete_mesh_client->sendGoal(goal,  boost::bind(&RosGraspitInterface::completeMeshCB, this, _1, _2),
                actionlib::SimpleActionClient<graspit_shape_completion::CompleteMeshAction>::SimpleActiveCallback(),
                actionlib::SimpleActionClient<graspit_shape_completion::CompleteMeshAction>::SimpleFeedbackCallback());
}



void RosGraspitInterface::completeMeshCB(const actionlib::SimpleClientGoalState& state,
                       const graspit_shape_completion::CompleteMeshResultConstPtr& result)
{
    ROS_INFO("Sucessfully recieved completed mesh");
    target_total_time->setText(QString("Target Total Time(ms): ") + QString::number(result->total_time));
    target_preprocess_time->setText(QString("Target Pre-processing Time(ms): ") + QString::number(result->preprocess_time));
    target_completion_time->setText(QString("Target Completion Time(ms): ") + QString::number(result->completion_time));
    target_postprocess_time->setText(QString("Target Post-processing Time(ms): ") + QString::number(result->postprocess_time));

    transf t = selected_body->getTran();

    geometry_msgs::Vector3 offset = geometry_msgs::Vector3();

    offset.x = t.translation().x();
    offset.y = t.translation().y();
    offset.z = t.translation().z();

    //remove the object we are completing
    //graspItGUI->getMainWorld()->deselectElement(selected_body);
    graspItGUI->getMainWorld()->deselectAll();
    graspItGUI->getMainWorld()->destroyElement(selected_body, true);


    int new_mesh_index = graspItGUI->getMainWorld()->getNumGB() + 1;
    addMesh(new_mesh_index, result->completed_mesh, offset);
}


}
