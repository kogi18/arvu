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
//#include <cstring>


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
private:
	void emptyCurrentImg();
	void renderBackgroundGL();
	GLuint textureId;
	GLuint objectTextureId;
	float *vertexList = NULL, *normalList = NULL, *textureList = NULL;
	int totalVertexCount = 0;
	boost::mutex dataMutex; //protects current_img
	cv::Mat current_img;
};

#endif /* ARVIEWER_H_ */
