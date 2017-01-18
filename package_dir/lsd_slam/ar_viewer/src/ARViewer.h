/*
 * ARViewer.h
 *
 * AR - Exercise
 *
 */

#ifndef ARVIEWER_H_
#define ARVIEWER_H_

#include "QGLViewer/qglviewer.h"
#include "ar_viewer/keyframeMsg.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "boost/thread.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "ar_viewer/keyframeMsg.h"
#include "iostream" //complex object textures require strings
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include "KeyFrameDisplay.h"

class ARViewer: public QGLViewer {
public:
	ARViewer();
	virtual ~ARViewer();
	void setImage(const cv::Mat& image);
	void addPoseMsg(geometry_msgs::PoseStampedConstPtr msg);
	void addFrameMsg(ar_viewer::keyframeMsgConstPtr msg); // has more information about frame eg camToWorld matrix
protected:
	virtual void draw();
	virtual void drawCube();
	virtual void drawComplex();
	virtual void init();
	virtual void initComplexModel();
	virtual void drawGround();
private:
	void emptyCurrentImg();
	void renderBackgroundGL();
	GLuint textureId;
	GLuint objectTextureId;
	const char* complexObjFile;
	const char* complexTexFile;
	float *vertexList = NULL, *normalList = NULL, *textureList = NULL;
	int totalVertexCount = 0;
	boost::mutex dataMutex; //protects current_img
	cv::Mat current_img;
	// animation settings
	int circlingMax, straightMovementMax, currentFrame; // time/frame triggers and time/frame coutner
	float circlingRotStepDeg, circlingRotStepRad, straightSpeedX, straightSpeedZ, droppingSpeedY;
	float circlingBufferX, circlingRadius;
	int isCirgling, isFirstHalf; // 'booleans'
	float CO_x, CO_y, CO_z; //complex object positions
	float CO_Deg_y; //complex object rotations
	float CUBE_x, CUBE_y, CUBE_z; //cube object positions
	float PI = 3.1415926535897;
	float deg2rad = PI / 180;
	// keyframes for 2.0
	KeyFrameDisplay* kfd;

	// booleans
	int mustDrawObjects, mustDrawMesh, mustDrawGround, mustDrawPC;
};

#endif /* ARVIEWER_H_ */
