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

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <include/EGPlanner/egPlanner.h>
#include <include/EGPlanner/simAnnPlanner.h>


#include <include/grasp.h>
#include <include/triangle.h>
#include <geometry_msgs/Point.h>
#include <QtGui>
#include <thread>

#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>

namespace graspit_ros_planning
{
using namespace graspit_msgs;
void meshMsgToVerticesTriangles(const shape_msgs::Mesh &mesh,
                                std::vector<position> *vertices,
                                std::vector<int> *triangles,
                                pcl::PolygonMesh &pcl_mesh)
{
    pcl::PointCloud<pcl::PointXYZ> *pcl_temp = new pcl::PointCloud<pcl::PointXYZ>();
    //pcl::PCLPointCloud2::Ptr pcl_pc2 (new pcl::PCLPointCloud2());

    for(int i=0 ; i < mesh.vertices.size();i++)
    {
        geometry_msgs::Point p = mesh.vertices.at(i);
        vertices->push_back(position(p.x,p.y,p.z));
        //pcl_mesh.cloud.data.push_back(new pcl::PointXYZ(p.x,p.y,p.z));
        pcl::PointXYZ *point = new pcl::PointXYZ(p.x,p.y,p.z);
        pcl_temp->push_back(*point);
    }

    pcl::toPCLPointCloud2(*pcl_temp, pcl_mesh.cloud);

    for(int i=0 ; i < mesh.triangles.size();i++)
    {
        shape_msgs::MeshTriangle t = mesh.triangles.at(i);

        triangles->push_back(t.vertex_indices.at(0));
        triangles->push_back(t.vertex_indices.at(1));
        triangles->push_back(t.vertex_indices.at(2));

        pcl::Vertices* p = new pcl::Vertices;
        p->vertices.push_back( t.vertex_indices.at(0));
        p->vertices.push_back( t.vertex_indices.at(1));
        p->vertices.push_back( t.vertex_indices.at(2));
        pcl_mesh.polygons.push_back(*p);

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
    recognizeObjectsActionClient = new actionlib::SimpleActionClient<RunObjectRecognitionAction>("/recognize_objects_action", true);

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
    QPushButton * objectRecButton = new QPushButton("ObjectRec");

    captureSceneButton->setDefault(true);
    shapeCompletionButton->setDefault(true);
    objectRecButton->setDefault(true);

    QWidget *shapeCompletionControlBox = new QWidget();

    scene_segmentation_time = new QLabel(tr("Scene Segmentation Time:?"));
    target_completion_time = new QLabel(tr("Target Completion Time:?"));

    QGridLayout *mainLayout = new QGridLayout;
    mainLayout->addWidget(scene_segmentation_time, 0, 0);
    mainLayout->addWidget(target_completion_time, 1,0);
    mainLayout->addWidget(captureSceneButton, 0, 1, 1, 2);
    mainLayout->addWidget(shapeCompletionButton, 1, 1, 1, 2);
    mainLayout->addWidget(objectRecButton, 2, 1, 1, 2);

    shapeCompletionControlBox->setLayout(mainLayout);


    //shapeCompletionControlBox->resize(QSize(200,100));

    shapeCompletionControlBox->show();

    QObject::connect(captureSceneButton, SIGNAL(clicked()), this, SLOT(onCaptureSceneButtonPressed()));
    QObject::connect(shapeCompletionButton, SIGNAL(clicked()), this, SLOT(onCompleteShapeButtonPressed()));
    QObject::connect(objectRecButton, SIGNAL(clicked()), this, SLOT(onObjectRecButtonButtonPressed()));


    ROS_INFO("Graspit Shape Completion Plugin Successfully Initialized");
    return 0;
}

int RosGraspitInterface::mainLoop()
{
    ros::spinOnce();
    return 0;
}


void RosGraspitInterface::onObjectRecButtonButtonPressed()
{
    ROS_INFO("Sending ObjectRec Goal");
    RunObjectRecognitionGoal goal;
    recognizeObjectsActionClient->sendGoal(goal, boost::bind(&RosGraspitInterface::objectRecognitionCB, this, _1, _2),
                                          actionlib::SimpleActionClient<graspit_msgs::RunObjectRecognitionAction>::SimpleActiveCallback(),
                                          actionlib::SimpleActionClient<graspit_msgs::RunObjectRecognitionAction>::SimpleFeedbackCallback());
    ROS_INFO("Sent ObjectRec Goal; Waiting for Response");
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
    scene_segmentation_time->setText(QString(result->completion_time));
    ROS_INFO("Sucessfully recieved meshed scene");
}


void RosGraspitInterface::addMesh(int mesh_index, shape_msgs::Mesh mesh, geometry_msgs::Vector3 offset)
{
    std::vector<int> triangles;
    std::vector<position> vertices;
    pcl::PolygonMesh *pcl_mesh = new pcl::PolygonMesh;
    meshMsgToVerticesTriangles(mesh, &vertices, &triangles, *pcl_mesh);

    GraspableBody *b = new GraspableBody(graspItGUI->getMainWorld());
    if (b->loadGeometryMemory(vertices, triangles) == SUCCESS)
    {
        ROS_INFO("setting mesh name\n");
        QString meshname = QString("mesh_") +  QString::number(mesh_index);
        b->setName(meshname);
        b->setMaterial(5);\
        b->setMass(200);

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

        b->setFilename(QString("models/objects/") + b->getName() + ".xml");
        QString mesh_filename =  b->getName() + QString(".ply");
        QString mesh_fullfilepath = QString(getenv("GRASPIT")) + QString("/models/objects/") +  mesh_filename;
        ROS_INFO("Savingin mesh file: %s",mesh_fullfilepath.toStdString().c_str());
        pcl::io::savePLYFileBinary(mesh_fullfilepath.toStdString().c_str(), *pcl_mesh);

        ROS_INFO("saving .xml");
        QFile file(b->getFilename());
        file.open(QIODevice::WriteOnly | QIODevice::Text);
        ROS_INFO("opened .xml");
        QTextStream stream(&file);
        ROS_INFO("about to stream .xml");
        stream << "<?xml version=\"1.0\" ?>";
        stream << "<root>";

        stream << "       <material>glass</material>\n";
        stream << "       <mass>200</mass>\n";
        stream << "        <cog>0 0 0</cog>\n";
        stream << "       <inertia_matrix>1584.3 -17.107 0.0 -17.107 1910.6  0.0 0.0 0.0 1746.5</inertia_matrix>\n";
        stream << "         <geometryFile type=\"ply\">\n";
        stream <<  mesh_filename.toStdString().c_str();
        stream << "</geometryFile>";
        stream << " </root>";

        //b->saveToXml(stream);
        ROS_INFO("wrote .xml");
        file.close();
        ROS_INFO("close .xml");
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
    target_completion_time->setText(QString(result->completion_time));

    transf t = selected_body->getTran();

    geometry_msgs::Vector3 offset = geometry_msgs::Vector3();

    offset.x = t.translation().x();
    offset.y = t.translation().y();
    offset.z = t.translation().z();

    //remove the object we are completing
    graspItGUI->getMainWorld()->deselectAll();
    graspItGUI->getMainWorld()->destroyElement(selected_body, true);

    int new_mesh_index = graspItGUI->getMainWorld()->getNumGB() + 1;
    addMesh(new_mesh_index, result->completed_mesh, offset);
}

void RosGraspitInterface::objectRecognitionCB(const actionlib::SimpleClientGoalState& state,
                                                    const graspit_msgs::RunObjectRecognitionResultConstPtr& result)
{
    std::for_each(result->object_info.begin(),
                  result->object_info.end(),
                  boost::bind(&RosGraspitInterface::addObject, this, _1));
    ROS_INFO("Sucessfully Finished runObjectRecognition Request");
}

void RosGraspitInterface::addObject(graspit_msgs::ObjectInfo object)
{
    QString modelName(QString::fromStdString(object.model_name));
    QString objectName(QString::fromStdString(object.object_name));
    transf object_pose = transf(
                Quaternion(
                    object.object_pose.orientation.w,
                    object.object_pose.orientation.x,
                    object.object_pose.orientation.y,
                    object.object_pose.orientation.z),
                vec3(
                    object.object_pose.position.x*1000.0,
                    object.object_pose.position.y*1000.0,
                    object.object_pose.position.z*1000.0
                    ));
    ROS_INFO("Adding Model %s", modelName.toStdString().c_str());
    ROS_INFO("Adding Model %s", objectName.toStdString().c_str());
    addToWorld(modelName, objectName, object_pose);
}

void RosGraspitInterface::addToWorld(const QString modelname, const QString object_name, const transf object_pose)
{
    QString model_filename = modelname + QString(".xml");
    ROS_INFO("model filename: %s" , model_filename.toStdString().c_str());

    QString body_file = QString(getenv("GRASPIT")) + "/" + "models/objects/" + model_filename;
    Body *b = graspItGUI->getIVmgr()->getWorld()->importBody("GraspableBody", body_file);
    if(!b)
    {
        QString body_file = QString(getenv("GRASPIT")) + "/" + "models/object_database/" + model_filename;
        b = graspItGUI->getIVmgr()->getWorld()->importBody("GraspableBody", body_file);
    }
    if(b)
    {
        b->setTran(object_pose);
        b->setName(object_name);
        b->setObjectName(object_name);
    }

}

}
