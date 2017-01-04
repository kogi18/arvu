/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam> 
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with dvo. If not, see <http://www.gnu.org/licenses/>.
*/


#include "ros/ros.h"
#include "boost/thread.hpp"
#include "settings.h"
#include "PointCloudViewer.h"
#include "ARViewer.h"
#include "ARWorker.h"
#include "IOWrapper/NotifyBuffer.h"
#include "IOWrapper/TimestampedObject.h"
#include "IOWrapper/ROS/ROSImageStreamThread.h"
#include <dynamic_reconfigure/server.h>
#include "ar_viewer/ARViewerParamsConfig.h"
#include <qapplication.h>


#include "ar_viewer/keyframeGraphMsg.h"
#include "ar_viewer/keyframeMsg.h"

#include "geometry_msgs/PoseStamped.h"

namespace lsd_slam{
	std::string packagePath = "";
}

using lsd_slam::TimestampedMat;
using lsd_slam::NotifyBuffer;
ARViewer* viewer = nullptr;
ARWorker* worker = nullptr;
NotifyBuffer<TimestampedMat>* imagebuffer = nullptr;

void dynConfCb(ar_viewer::ARViewerParamsConfig &config, uint32_t level)
{

	pointTesselation = config.pointTesselation;
	lineTesselation = config.lineTesselation;

	keepInMemory = config.keepInMemory;
	showKFCameras = config.showKFCameras;
	showKFPointclouds = config.showKFPointclouds;
	showConstraints = config.showConstraints;
	showCurrentCamera = config.showCurrentCamera;
	showCurrentPointcloud = config.showCurrentPointcloud;


	scaledDepthVarTH = exp10( config.scaledDepthVarTH );
	absDepthVarTH = exp10( config.absDepthVarTH );
	minNearSupport = config.minNearSupport;
	sparsifyFactor = config.sparsifyFactor;
	cutFirstNKf = config.cutFirstNKf;

	saveAllVideo = config.saveAllVideo;

}

void poseCb(geometry_msgs::PoseStampedConstPtr msg)
{
	if(viewer != 0)
		viewer->addPoseMsg(msg);
}
void frameCb(ar_viewer::keyframeMsgConstPtr msg)
{
	if(msg->time > lastFrameTime) return;

	if(viewer != 0)
		viewer->addFrameMsg(msg);
}
void graphCb(ar_viewer::keyframeGraphMsgConstPtr msg)
{
//	if(worker != 0)
//		worker->addGraphMsg(msg);
}


void rosThreadLoop( int argc, char** argv )
{
	printf("Started ROS thread\n");
	//glutInit(&argc, argv);
	ros::init(argc, argv, "viewer");
	ROS_INFO("ar_viewer started");
	dynamic_reconfigure::Server<ar_viewer::ARViewerParamsConfig> srv;
	srv.setCallback(dynConfCb);

	ros::NodeHandle nh;
	ros::Subscriber pose_sub = nh.subscribe(nh.resolveName("lsd_slam/pose"),1, poseCb);
	ros::Subscriber liveFrames_sub = nh.subscribe(nh.resolveName("lsd_slam/liveframes"),1, frameCb);
	ros::Subscriber keyFrames_sub = nh.subscribe(nh.resolveName("lsd_slam/keyframes"),20, frameCb);
	ros::Subscriber graph_sub       = nh.subscribe(nh.resolveName("lsd_slam/graph"),10, graphCb);

	ros::spin();
	ros::shutdown();

	printf("Exiting ROS thread\n");

	exit(1);
}

void workerThreadLoop( )
{
	if(worker)
		worker->Loop();

	printf("Exiting Cam thread\n");

	exit(1);
}

int main( int argc, char** argv )
{


	// start ROS thread waits for input at ROS ports and executes the callbacks
	boost::thread rosThread = boost::thread(rosThreadLoop, argc, argv);
	printf("Started QApplication thread\n");
	QApplication application(argc,argv);

	lsd_slam::ROSImageStreamThread imagesource = lsd_slam::ROSImageStreamThread();
	std::string calibFile;
	setlocale(LC_NUMERIC, "C");
	if(ros::param::get("~calib", calibFile))
	{
		ros::param::del("~calib");
		imagesource.setCalibration(calibFile);
	}
	else{
		imagesource.setCalibration("");
	}
	viewer = new ARViewer();
	worker = new ARWorker(viewer, &imagesource);
	//Worker Thread waits on input from imagebuffer and gets input from the ROSthread
	boost::thread workerThread = boost::thread(workerThreadLoop);

	// Instantiate the viewer.
	//viewer = new PointCloudViewer();
	#if QT_VERSION < 0x040000
		// Set the viewer as the application main widget.
		application.setMainWidget(viewer);
	#else
		viewer->setWindowTitle("AR Viewer");
	#endif

	// Make the viewer window visible on screen.
	viewer->show();


	application.exec();


	printf("Shutting down... \n");
	ros::shutdown();
	rosThread.join();
	printf("Done. \n");
}
