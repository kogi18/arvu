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

#define GL_GLEXT_PROTOTYPES 1

#include "KeyFrameDisplay.h"
#include <stdio.h>
#include "settings.h"

#include <GL/glx.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include "opencv2/opencv.hpp"

#include "ros/package.h"

KeyFrameDisplay::KeyFrameDisplay()
{
	originalInput = 0;
	id = 0;
	vertexBufferIdValid = false;
	glBuffersValid = false;
	depthMapValid = false;

	camToWorld = Sophus::Sim3f();
	width=height=0;

	my_scaledTH = my_absTH = 0;

	totalPoints = displayedPoints = 0;
//	cv::namedWindow("Depth map");// Create a window for display.
//	cv::namedWindow("Scaled depth map", cv::WINDOW_NORMAL);// Create a window for display.
//	cv::resizeWindow("Scaled depth map",400, 300);
	cv::namedWindow("Inpainted depth map", cv::WINDOW_NORMAL);// Create a window for display.
	cv::resizeWindow("Inpainted depth map",640, 480);
	depthMapHeight = 15;
	depthMapWidth = 20;
	mustDrawMesh = 0;
}


KeyFrameDisplay::~KeyFrameDisplay()
{
	if(vertexBufferIdValid)
	{
		glDeleteBuffers(1, &vertexBufferId);
		vertexBufferIdValid = false;
	}

	if(originalInput != 0)
		delete[] originalInput;

	if(mustDrawMesh > 0){
		inpainted_depth_img.release();
	}
}

cv::Vec3b KeyFrameDisplay::getVisualizationColor(float idepth) const{
	float r = (0-idepth) * 255 / 1.0; if(r < 0) r = -r;
	float g = (1-idepth) * 255 / 1.0; if(g < 0) g = -g;
	float b = (2-idepth) * 255 / 1.0; if(b < 0) b = -b;
	
	uchar rc = r < 0 ? 0 : (r > 255 ? 255 : r);
	uchar gc = g < 0 ? 0 : (g > 255 ? 255 : g);
	uchar bc = b < 0 ? 0 : (b > 255 ? 255 : b);

	return cv::Vec3b(255-rc, 255-gc, 255-bc);
}

float KeyFrameDisplay::color2Depth(cv::Vec3b color){
	float idepth = 0;
	if(color[2] > 0){
		idepth = 2.0f - (255 - color[2])/255.0f;
	}
	
	if(idepth == 0 && color[1] > 0){
		idepth = 1.0f - (255 - color[1])/255.0f;
	}

	if(idepth == 0 && color[0] > 0){
		idepth = (255 - color[0])/255.0f;
	}

	if(idepth == 0){
		return 0.0f;
	}

	if(idepth < 0){
		idepth = -idepth;
	}

	return 1.0f/idepth;
}


void KeyFrameDisplay::setFrom(ar_viewer::keyframeMsgConstPtr msg)
{
	// copy over campose.
	memcpy(camToWorld.data(), msg->camToWorld.data(), 7*sizeof(float));

	fx = msg->fx;
	fy = msg->fy;
	cx = msg->cx;
	cy = msg->cy;

	fxi = 1/fx;
	fyi = 1/fy;
	cxi = -cx / fx;
	cyi = -cy / fy;

	width = msg->width;
	height = msg->height;
	id = msg->id;
	time = msg->time;

	if(originalInput != 0)
		delete[] originalInput;
	originalInput=0;

	if(msg->pointcloud.size() != width*height*sizeof(InputPointDense))
	{
		if(msg->pointcloud.size() != 0)
		{
			printf("WARNING: PC with points, but number of points not right! (is %zu, should be %u*%dx%d=%u)\n",
					msg->pointcloud.size(), sizeof(InputPointDense), width, height, width*height*sizeof(InputPointDense));
		}
	}
	else
	{
		originalInput = new InputPointDense[width*height];
		memcpy(originalInput, msg->pointcloud.data(), width*height*sizeof(InputPointDense));
	}
	glBuffersValid = false;

	if(mustDrawMesh > 0){
		cv::Mat depth_img = cv::Mat::zeros(height, width, CV_8UC3);
		cv::Mat scaled_depth_img = cv::Mat::zeros(depthMapHeight, depthMapWidth, CV_8UC3);
		cv::Mat scaled_border_depth_img = cv::Mat::zeros(depthMapHeight+1, depthMapWidth+1, CV_8UC3);
		cv::Mat inpainted_border_depth_img = cv::Mat::zeros(depthMapHeight+1, depthMapWidth+1, CV_8UC3);

		inpainted_depth_img = cv::Mat::zeros(depthMapHeight, depthMapWidth, CV_8UC3);

		float idepth;
		// set the colors
		for(int y=0;y<height;y++){
			for(int x=0;x<width;x++){
				idepth = originalInput[x+y*width].idepth;
				cv::Vec3b color = getVisualizationColor(idepth);
				depth_img.at<cv::Vec3b>(y, x) = color;
			}
		}
		// scale down the depth map
		cv::resize(depth_img,scaled_border_depth_img,cv::Size(scaled_border_depth_img.cols, scaled_border_depth_img.rows), cv::INTER_CUBIC); //linear has big losses in shrinking
		cv::Mat depth_mask_img = cv::Mat::zeros(scaled_border_depth_img.rows, scaled_border_depth_img.cols,CV_8UC1);
		// generate the mask for inpaint
		int treshold = 25;
		for(int y=0;y<scaled_border_depth_img.rows;y++){
			for(int x=0;x<scaled_border_depth_img.cols;x++){
				cv::Vec3b color = scaled_border_depth_img.at<cv::Vec3b>(y, x);
				if(color[0] < treshold && color[1] < treshold && color[2] < treshold){
	//			if(color[0] == 0 && color[1] == 0 && color[2] == 0){
					depth_mask_img.at<uchar>(y, x) = 255;
				}
			}
		}
		// do the inpaint
		inpaint(scaled_border_depth_img, depth_mask_img, inpainted_border_depth_img, 2, cv::INPAINT_TELEA);

		//remove the border - inpainting fails at left and upper border, so we do it on 1px higher and wider image than remove the first row and column
		//int blackCount = 0;
		for(int y=0;y<scaled_depth_img.rows;y++){
			for(int x=0;x<scaled_depth_img.cols;x++){
				scaled_depth_img.at<cv::Vec3b>(y, x) =  scaled_border_depth_img.at<cv::Vec3b>(y+1, x+1);
				cv::Vec3b color = inpainted_border_depth_img.at<cv::Vec3b>(y+1, x+1);
				inpainted_depth_img.at<cv::Vec3b>(y, x) = color;
				/*
				if(color[0] == 0 && color[1] == 0 && color[2] == 0){
					blackCount++;
				}
				*/
			}
		}
		//std::cout << 'B' << blackCount << std::endl;
		depthMapValid = true;

	 //   cv::imshow( "Depth map", depth_img );
	 //   cv::imshow( "Scaled depth map", scaled_depth_img );
		cv::imshow("Inpainted depth map", inpainted_depth_img);

		depth_mask_img.release();
		scaled_border_depth_img.release();
		inpainted_border_depth_img.release();
		depth_img.release();
		scaled_depth_img.release();
	}
	else{
		// have to close the window if not used
		cv::destroyWindow("Inpainted depth map");
	}
}

void KeyFrameDisplay::drawMesh(float alpha)
{
	float halfWidth = depthMapWidth / 2.0f, halfHeight = depthMapHeight / 2.0f;
	float x1, x2, y1, y2;

	if(depthMapValid){
		for(int y=0;y<depthMapHeight-1;y++){
			for(int x=0;x<depthMapWidth-1;x++){
				x1 = (x - halfWidth) / halfWidth;
				x2 = (x + 1 - halfWidth) / halfWidth;
				y1 = (halfHeight - y) / halfHeight;
				y2 = (halfHeight - y -1) / halfHeight;
				cv::Vec3b colorX1Y1 = inpainted_depth_img.at<cv::Vec3b>(y, x);
				cv::Vec3b colorX1Y2 = inpainted_depth_img.at<cv::Vec3b>(y+1, x);
				cv::Vec3b colorX2Y1 = inpainted_depth_img.at<cv::Vec3b>(y, x+1);
				cv::Vec3b colorX2Y2 = inpainted_depth_img.at<cv::Vec3b>(y+1, x+1);


				glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
				glEnable(GL_BLEND);

				glBegin(GL_QUADS);  
				glNormal3f(0.0f, 0.0f, 1.0f); 

				glColor4f(colorX1Y1[2] / 255.0f, colorX1Y1[1] / 255.0f, colorX1Y1[0] / 255.0f, alpha);
				glVertex3f(x1, y1, -1*color2Depth(colorX1Y1));
				glColor4f(colorX1Y2[2] / 255.0f, colorX1Y2[1] / 255.0f, colorX1Y2[0] / 255.0f, alpha);
				glVertex3f(x1, y2, -1*color2Depth(colorX1Y2));
				glColor4f(colorX2Y2[2] / 255.0f, colorX2Y2[1] / 255.0f, colorX2Y2[0] / 255.0f, alpha);
				glVertex3f(x2, y2, -1*color2Depth(colorX2Y2));
				glColor4f(colorX2Y1[2] / 255.0f, colorX2Y1[1] / 255.0f, colorX2Y2[0] / 255.0f, alpha);
				glVertex3f(x2, y1, -1*color2Depth(colorX2Y1));
				glEnd();

				glDisable(GL_BLEND);

				//wireframe
				glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
				glBegin(GL_QUADS); 
				glNormal3f(0.0f, 0.0f, 1.0f); 
				glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
				glVertex3f(x1, y1, -1*color2Depth(colorX1Y1));
				glVertex3f(x1, y2, -1*color2Depth(colorX1Y2));
				glVertex3f(x2, y2, -1*color2Depth(colorX2Y2));
				glVertex3f(x2, y1, -1*color2Depth(colorX2Y1));
				glEnd();
				glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );

			}
		}
	}

}

void KeyFrameDisplay::refreshPC()
{
//	minNearSupport = 9;
	bool paramsStillGood = my_scaledTH == scaledDepthVarTH &&
			my_absTH == absDepthVarTH &&
			my_scale*1.2 > camToWorld.scale() &&
			my_scale < camToWorld.scale()*1.2 &&
			my_minNearSupport == minNearSupport &&
			my_sparsifyFactor == sparsifyFactor;



	if(glBuffersValid && (paramsStillGood || numRefreshedAlready > 10)) return;
	numRefreshedAlready++;

	glBuffersValid = true;


	// delete old vertex buffer
	if(vertexBufferIdValid)
	{
		glDeleteBuffers(1, &vertexBufferId);
		vertexBufferIdValid = false;
	}



	// if there are no vertices, done!
	if(originalInput == 0)
		return;


	// make data
	MyVertex* tmpBuffer = new MyVertex[width*height];

	my_scaledTH =scaledDepthVarTH;
	my_absTH = absDepthVarTH;
	my_scale = camToWorld.scale();
	my_minNearSupport = minNearSupport;
	my_sparsifyFactor = sparsifyFactor;
	// data is directly in ros message, in correct format.
	vertexBufferNumPoints = 0;

	int total = 0, displayed = 0;
	for(int y=1;y<height-1;y++)
		for(int x=1;x<width-1;x++)
		{
			if(originalInput[x+y*width].idepth <= 0) continue;
			total++;


			if(my_sparsifyFactor > 1 && rand()%my_sparsifyFactor != 0) continue;

			float depth = 1 / originalInput[x+y*width].idepth;
			float depth4 = depth*depth; depth4*= depth4;


			if(originalInput[x+y*width].idepth_var * depth4 > my_scaledTH)
				continue;

			if(originalInput[x+y*width].idepth_var * depth4 * my_scale*my_scale > my_absTH)
				continue;

			if(my_minNearSupport > 1)
			{
				int nearSupport = 0;
				for(int dx=-1;dx<2;dx++)
					for(int dy=-1;dy<2;dy++)
					{
						int idx = x+dx+(y+dy)*width;
						if(originalInput[idx].idepth > 0)
						{
							float diff = originalInput[idx].idepth - 1.0f / depth;
							if(diff*diff < 2*originalInput[x+y*width].idepth_var)
								nearSupport++;
						}
					}

				if(nearSupport < my_minNearSupport)
					continue;
			}

			tmpBuffer[vertexBufferNumPoints].point[0] = (x*fxi + cxi) * depth; //1.0f / float(width/2 - x);
			tmpBuffer[vertexBufferNumPoints].point[1] = -(y*fyi + cyi) * depth; // 1.0f / float(height/2 - y);
			tmpBuffer[vertexBufferNumPoints].point[2] = -depth;

			cv::Vec3b color = getVisualizationColor(originalInput[x+y*width].idepth);
			//std::cout << int(originalInput[x+y*width].color[0]) << ',' << int(originalInput[x+y*width].color[1]) << ',' <<  int(originalInput[x+y*width].color[2]) << std::endl;

			tmpBuffer[vertexBufferNumPoints].color[3] = 100;
			tmpBuffer[vertexBufferNumPoints].color[2] = color[0];
			tmpBuffer[vertexBufferNumPoints].color[1] = color[1];
			tmpBuffer[vertexBufferNumPoints].color[0] = color[2];

			vertexBufferNumPoints++;
			displayed++;
		}
	totalPoints = total;
	displayedPoints = displayed;

	// create new ones, static
	vertexBufferId=0;
	glGenBuffers(1, &vertexBufferId);
	glBindBuffer(GL_ARRAY_BUFFER, vertexBufferId);         // for vertex coordinates
	glBufferData(GL_ARRAY_BUFFER, sizeof(MyVertex) * vertexBufferNumPoints, tmpBuffer, GL_STATIC_DRAW);
	vertexBufferIdValid = true;



	if(!keepInMemory)
	{
		delete[] originalInput;
		originalInput = 0;
	}




	delete[] tmpBuffer;
}



void KeyFrameDisplay::drawCam(float lineWidth, float* color)
{
	if(width == 0)
		return;


	glPushMatrix();

		Sophus::Matrix4f m = camToWorld.matrix();
		glMultMatrixf((GLfloat*)m.data());

		if(color == 0)
			glColor3f(1,0,0);
		else
			glColor3f(color[0],color[1],color[2]);

		glLineWidth(lineWidth);
		glBegin(GL_LINES);
		glVertex3f(0,0,0);
		glVertex3f(0.05*(0-cx)/fx,0.05*(0-cy)/fy,0.05);
		glVertex3f(0,0,0);
		glVertex3f(0.05*(0-cx)/fx,0.05*(height-1-cy)/fy,0.05);
		glVertex3f(0,0,0);
		glVertex3f(0.05*(width-1-cx)/fx,0.05*(height-1-cy)/fy,0.05);
		glVertex3f(0,0,0);
		glVertex3f(0.05*(width-1-cx)/fx,0.05*(0-cy)/fy,0.05);

		glVertex3f(0.05*(width-1-cx)/fx,0.05*(0-cy)/fy,0.05);
		glVertex3f(0.05*(width-1-cx)/fx,0.05*(height-1-cy)/fy,0.05);

		glVertex3f(0.05*(width-1-cx)/fx,0.05*(height-1-cy)/fy,0.05);
		glVertex3f(0.05*(0-cx)/fx,0.05*(height-1-cy)/fy,0.05);

		glVertex3f(0.05*(0-cx)/fx,0.05*(height-1-cy)/fy,0.05);
		glVertex3f(0.05*(0-cx)/fx,0.05*(0-cy)/fy,0.05);

		glVertex3f(0.05*(0-cx)/fx,0.05*(0-cy)/fy,0.05);
		glVertex3f(0.05*(width-1-cx)/fx,0.05*(0-cy)/fy,0.05);

		glEnd();
	glPopMatrix();
}

int KeyFrameDisplay::flushPC(std::ofstream* f)
{

	MyVertex* tmpBuffer = new MyVertex[width*height];
	int num = 0;
	for(int y=1;y<height-1;y++)
		for(int x=1;x<width-1;x++)
		{
			if(originalInput[x+y*width].idepth <= 0) continue;

			if(my_sparsifyFactor > 1 && rand()%my_sparsifyFactor != 0) continue;

			float depth = 1 / originalInput[x+y*width].idepth;
			float depth4 = depth*depth; depth4*= depth4;

			if(originalInput[x+y*width].idepth_var * depth4 > my_scaledTH)
				continue;

			if(originalInput[x+y*width].idepth_var * depth4 * my_scale*my_scale > my_absTH)
				continue;

			if(my_minNearSupport > 1)
			{
				int nearSupport = 0;
				for(int dx=-1;dx<2;dx++)
					for(int dy=-1;dy<2;dy++)
					{
						int idx = x+dx+(y+dy)*width;
						if(originalInput[idx].idepth > 0)
						{
							float diff = originalInput[idx].idepth - 1.0f / depth;
							if(diff*diff < 2*originalInput[x+y*width].idepth_var)
								nearSupport++;
						}
					}

				if(nearSupport < my_minNearSupport)
					continue;
			}


			Sophus::Vector3f pt = camToWorld * (Sophus::Vector3f((x*fxi + cxi), (y*fyi + cyi), 1) * depth);
			tmpBuffer[num].point[0] = pt[0];
			tmpBuffer[num].point[1] = pt[1];
			tmpBuffer[num].point[2] = pt[2];



			tmpBuffer[num].color[3] = 100;
			tmpBuffer[num].color[2] = originalInput[x+y*width].color[0];
			tmpBuffer[num].color[1] = originalInput[x+y*width].color[1];
			tmpBuffer[num].color[0] = originalInput[x+y*width].color[2];

			num++;
		}




	for(int i=0;i<num;i++)
	{
		f->write((const char *)tmpBuffer[i].point,3*sizeof(float));
		float color = tmpBuffer[i].color[0] / 255.0;
		f->write((const char *)&color,sizeof(float));
	}
	//	*f << tmpBuffer[i].point[0] << " " << tmpBuffer[i].point[1] << " " << tmpBuffer[i].point[2] << " " << (tmpBuffer[i].color[0] / 255.0) << "\n";

	delete tmpBuffer;

	printf("Done flushing frame %d (%d points)!\n", this->id, num);
	return num;
}

void KeyFrameDisplay::drawPC(float pointSize, float alpha)
{
	refreshPC();

	if(!vertexBufferIdValid)
	{
		return;
	}

	GLfloat LightColor[] = {1, 1, 1, 1};
	if(alpha < 1)
	{
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		LightColor[0] = LightColor[1] = 0;
		glEnable(GL_LIGHTING);
		glDisable(GL_LIGHT1);

		glLightfv (GL_LIGHT0, GL_AMBIENT, LightColor);
	}
	else
	{
		glDisable(GL_LIGHTING);
	}


	glPushMatrix();

		//Sophus::Matrix4f m = camToWorld.matrix();
		//glMultMatrixf((GLfloat*)m.data());

		glPointSize(pointSize);

		glBindBuffer(GL_ARRAY_BUFFER, vertexBufferId);

		glVertexPointer(3, GL_FLOAT, sizeof(MyVertex), 0);
		glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(MyVertex), (const void*) (3*sizeof(float)));

		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_COLOR_ARRAY);

		glDrawArrays(GL_POINTS, 0, vertexBufferNumPoints);

		glDisableClientState(GL_COLOR_ARRAY);
		glDisableClientState(GL_VERTEX_ARRAY);

	glPopMatrix();




	if(alpha < 1)
	{
		glDisable(GL_BLEND);
		glDisable(GL_LIGHTING);
		LightColor[2] = LightColor[1] = LightColor[0] = 1;
		glLightfv (GL_LIGHT0, GL_AMBIENT_AND_DIFFUSE, LightColor);
	}
}

