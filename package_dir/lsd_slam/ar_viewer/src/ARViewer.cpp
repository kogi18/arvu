/*
 * ARViewer.cpp
 *
 * AR - Exercise
 */

#include "ARViewer.h"
#define DEBUG(s) std::cout << __FUNCTION__ << " " << s << std::endl;

ARViewer::ARViewer(){
	/*
	 * Exercise 1.2: initialize a color image as the background image (current_img)
	 */
	//TODO
	emptyCurrentImg();
}

ARViewer::~ARViewer() {

}

void ARViewer::emptyCurrentImg(){
	// set to be green
	current_img = cv::Mat(480,640,CV_8UC3 ,cv::Scalar(0,127,0));
}

void ARViewer::setImage(const cv::Mat& image){
	boost::lock_guard<boost::mutex> guard(dataMutex); // unlocks the dataMutex at function end
	/*
	 * Exercise 1.2:
	 */
	//TODO
	cv::Mat buffer(480,640,0);
	cv::flip(image,buffer,0);
	cv::cvtColor(buffer, current_img, CV_GRAY2RGB);
	buffer.release();
	DEBUG(current_img.type());
}

void ARViewer::addPoseMsg(geometry_msgs::PoseStampedConstPtr msg){
	/*
	 * Exercise 1.1: Update camera pose and orientation
	 */
//	std::cout << 'R'  << ' ' << msg->pose.orientation.x << ',' << msg->pose.orientation.y << ',' << msg->pose.orientation.z << ',' << msg->pose.orientation.w << std::endl;
//	std::cout << 'P'  << ' ' << msg->pose.position.x << ',' << msg->pose.position.y << ',' << msg->pose.position.z << std::endl;

	//TODO
	qglviewer::Vec pose = qglviewer::Vec(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
	qglviewer::Quaternion orientation = qglviewer::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);

	camera()->setPosition(pose);
	camera()->setOrientation(orientation);
}

void ARViewer::addFrameMsg(ar_viewer::keyframeMsgConstPtr msg){
// camToWorld seems to be [~Rx,~Ry,~Rz,~Rw,Tx,Ty], the rotations are the approximations from the image?
//	std::cout << 'F'  << ' ' << msg->camToWorld[0] << ',' << msg->camToWorld[1] << ',' << msg->camToWorld[2] << ',' << msg->camToWorld[3] << ',' << msg->camToWorld[4] << ',' << msg->camToWorld[5] << std::endl;
}

void ARViewer::drawCube(){
   glBegin(GL_QUADS);                // Begin drawing the color cube with 6 quads
	  // Top face (y = 1.0f)
	  // Define vertices in counter-clockwise (CCW) order with normal pointing out
	  glColor3f(0.0f, 1.0f, 0.0f);     // Green
	  glVertex3f( 1.0f, 1.0f, -1.0f);
	  glVertex3f(-1.0f, 1.0f, -1.0f);
	  glVertex3f(-1.0f, 1.0f,  1.0f);
	  glVertex3f( 1.0f, 1.0f,  1.0f);

	  // Bottom face (y = -1.0f)
	  glColor3f(1.0f, 0.5f, 0.0f);     // Orange
	  glVertex3f( 1.0f, -1.0f,  1.0f);
	  glVertex3f(-1.0f, -1.0f,  1.0f);
	  glVertex3f(-1.0f, -1.0f, -1.0f);
	  glVertex3f( 1.0f, -1.0f, -1.0f);

	  // Front face  (z = 1.0f)
	  glColor3f(1.0f, 0.0f, 0.0f);     // Red
	  glVertex3f( 1.0f,  1.0f, 1.0f);
	  glVertex3f(-1.0f,  1.0f, 1.0f);
	  glVertex3f(-1.0f, -1.0f, 1.0f);
	  glVertex3f( 1.0f, -1.0f, 1.0f);

	  // Back face (z = -1.0f)
	  glColor3f(1.0f, 1.0f, 0.0f);     // Yellow
	  glVertex3f( 1.0f, -1.0f, -1.0f);
	  glVertex3f(-1.0f, -1.0f, -1.0f);
	  glVertex3f(-1.0f,  1.0f, -1.0f);
	  glVertex3f( 1.0f,  1.0f, -1.0f);

	  // Left face (x = -1.0f)
	  glColor3f(0.0f, 0.0f, 1.0f);     // Blue
	  glVertex3f(-1.0f,  1.0f,  1.0f);
	  glVertex3f(-1.0f,  1.0f, -1.0f);
	  glVertex3f(-1.0f, -1.0f, -1.0f);
	  glVertex3f(-1.0f, -1.0f,  1.0f);

	  // Right face (x = 1.0f)
	  glColor3f(1.0f, 0.0f, 1.0f);     // Magenta
	  glVertex3f(1.0f,  1.0f, -1.0f);
	  glVertex3f(1.0f,  1.0f,  1.0f);
	  glVertex3f(1.0f, -1.0f,  1.0f);
	  glVertex3f(1.0f, -1.0f, -1.0f);
   glEnd();  // End of drawing color-cube
   //--------------------------------------------------------------------
}

void ARViewer::drawComplex(){
	// seperation for faster Exercise 1.3 implementation
	glBindTexture(GL_TEXTURE_2D, objectTextureId);

	glBegin(GL_TRIANGLES);
	glColor3f(1.0f, 1.0f, 1.0f);     // White
	for (int vertex = 0, row; vertex < totalVertexCount; vertex++) {
		row = vertex * 2;	// we look at each vertex as a row of floats in our Nx2 textureList
		glTexCoord2f(textureList[row], textureList[row + 1]); 
		row = vertex * 3;	// we look at each vertex as a row of floats in our Nx3 normalList/vertexList
		glNormal3f(normalList[row], normalList[row + 1], normalList[row + 2]); 
		glVertex3f(vertexList[row], vertexList[row + 1], vertexList[row + 2]);
	}
	glEnd();

	glBindTexture(GL_TEXTURE_2D, 0);
}

void ARViewer::draw(){

	glPushMatrix ();
	renderBackgroundGL();
	glPopMatrix ();
	
	glTranslatef(1.5f, 0.0f, -7.0f);  // Move right and into the screen
	
	drawCube();
	/*
	 * Exercise 1.3 - Replace the box with one or more complex 3D objects
	 */
	//TODOs
	glTranslatef(-5.0f, 0.0f, -10.0f);
	drawComplex();

}

void ARViewer::init(){

	glEnable(GL_TEXTURE_2D);
	glGenTextures(1, &textureId);
	glBindTexture(GL_TEXTURE_2D, textureId);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );

	glGenTextures(1, &objectTextureId);
	glBindTexture(GL_TEXTURE_2D, objectTextureId);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );

	initComplexModel();

	setAnimationPeriod(1000/30);
	startAnimation();

	camera()->setZClippingCoefficient(100);
	camera()->setZNearCoefficient(0.0001);
	camera()->setAspectRatio(640.0/480.0);
	camera()->setFieldOfView(0.785398*170.0/90.0*640.0/752.0);//0.785398); //vertical
}

void ARViewer::initComplexModel(){
	//code rearranged from my (Rok's) previous project

//	const char* complexObjFile = "/home/rok/rosbuild_ws/package_dir/lsd_slam/ar_viewer/assets/nukahedron.obj";
	const char* complexObjFile = "/home/rok/rosbuild_ws/package_dir/lsd_slam/ar_viewer/assets/vader.obj";
	const char* complexTexFile = "/home/rok/rosbuild_ws/package_dir/lsd_slam/ar_viewer/assets/vader.png";

	int maxV = 0,
		maxUV = 0,
		maxN = 0,
		maxF = 0,
		VIC = 0,
		UVIC = 0,
		NIC = 0,
		FIC = 0;

	// load OBJ file just for getting the table sizes a.k.a. first run
	char * line = new char[200], *token1, *token2, *token3;
	std::ifstream OBJ_FILE(complexObjFile);
	std::cout << " good()=" << OBJ_FILE.good();
	std::cout << " eof()=" << OBJ_FILE.eof();
	std::cout << " fail()=" << OBJ_FILE.fail();
	std::cout << " bad()=" << OBJ_FILE.bad();
	std::cerr << "Error: " << std::strerror(errno);

	//int counterI = 0;
	while (OBJ_FILE.good() && !OBJ_FILE.eof()) {
/*		counterI++;
		if(counterI > 80){
			break;
		}
		std::cout << counterI << std::endl;
*/		OBJ_FILE.getline(line, 200);
		std::cout << line << std::endl;

		if (std::strncmp(line, "f ", 2) == 0) {
			maxF += 1;

			std::strtok(line, " ");	// remove the f
			token1 = std::strtok(NULL, " "); // get 1st triplet of vertex/texture/normal
			token2 = std::strtok(NULL, " "); // get 2nd triplet of vertex/texture/normal
			token3 = std::strtok(NULL, " "); // get 3rd triplet of vertex/texture/normal

			// check indecies for 1st vertex
			maxV = std::max(maxV, (int) atof(std::strtok(token1, "/"))); // check vertex index
			maxUV = std::max(maxUV, (int)atof(std::strtok(NULL, "/"))); // check texture index
			maxN = std::max(maxN, (int)atof(std::strtok(NULL, "/"))); // check normal index
			// check indecies for 2nd vertex
			maxV = std::max(maxV, (int)atof(std::strtok(token2, "/"))); // check vertex index
			maxUV = std::max(maxUV, (int)atof(std::strtok(NULL, "/"))); // check texture index
			maxN = std::max(maxN, (int)atof(std::strtok(NULL, "/"))); // check normal index
			// check indecies for 3rd vertex
			maxV = std::max(maxV, (int)atof(std::strtok(token3, "/"))); // check vertex index
			maxUV = std::max(maxUV, (int)atof(std::strtok(NULL, "/"))); // check texture index
			maxN = std::max(maxN, (int)atof(std::strtok(NULL, "/"))); // check normal index
		}
	}
	std::cout << "Max indeces of f statements: V[" << maxV << "], UV[" << maxUV << "], N[" << maxN << "], f[" << maxF << "]" << std::endl;

	// load OBJ file
	float **OBJ_VERTICES = new float* [3];
	float **OBJ_NORMALS = new float*[3];
	float **OBJ_TEXUV = new float*[2];
	int **OBJ_TRIANGLE_VI = new int*[3];
	int **OBJ_TRIANGLE_NI = new int*[3];
	int **OBJ_TRIANGLE_TI = new int*[3];
	for (int axis = 0; axis < 3; axis++) {
		if (axis < 2) {
			OBJ_TEXUV[axis] = new float[maxUV];
		}
		OBJ_VERTICES[axis] = new float[maxV];
		OBJ_NORMALS[axis] = new float[maxN];
		OBJ_TRIANGLE_VI[axis] = new int[maxF];
		OBJ_TRIANGLE_NI[axis] = new int[maxF];
		OBJ_TRIANGLE_TI[axis] = new int[maxF];
	}

	// set the reader back to start
	OBJ_FILE.clear();
	OBJ_FILE.seekg(0, std::ios::beg);
	while(!OBJ_FILE.eof()){
		OBJ_FILE.getline(line,100);
		if (std::strncmp(line, "v ", 2) == 0) {
//			std::cout << line << std::endl;
			std::strtok(line, " ");	// remove the v
			OBJ_VERTICES[0][VIC] = (float)atof(std::strtok(NULL, " ")); // get the X
			OBJ_VERTICES[1][VIC] = (float)atof(std::strtok(NULL, " ")); // get the Y
			OBJ_VERTICES[2][VIC] = (float)atof(std::strtok(NULL, " ")); // get the Z
			VIC += 1;
		}
		else if (std::strncmp(line, "vn", 2) == 0) {
	//		std::cout << line << std::endl;
			std::strtok(line, " ");	// remove the vn
			OBJ_NORMALS[0][NIC] = (float)atof(std::strtok(NULL, " ")); // get the X
			OBJ_NORMALS[1][NIC] = (float)atof(std::strtok(NULL, " ")); // get the Y
			OBJ_NORMALS[2][NIC] = (float)atof(std::strtok(NULL, " ")); // get the Z
			NIC += 1;
		}
		else if (std::strncmp(line, "vt", 2) == 0) {
//			std::cout << line << std::endl;
			std::strtok(line, " ");	// remove the vt
			OBJ_TEXUV[0][UVIC] = (float)atof(std::strtok(NULL, " ")); // get the U
			OBJ_TEXUV[1][UVIC] = 1.0f - (float)atof(std::strtok(NULL, " ")); // get the V (image height goes other waz, therefore 1 - value makes it correct)
			UVIC += 1;
		}
		else if (std::strncmp(line, "f ", 2) == 0) {
	//		std::cout << line << std::endl;
			std::strtok(line, " ");	// remove the f
			token1 = std::strtok(NULL, " "); // get 1st triplet of vertex/texture/normal
			token2 = std::strtok(NULL, " "); // get 2nd triplet of vertex/texture/normal
			token3 = std::strtok(NULL, " "); // get 3rd triplet of vertex/texture/normal

			// save for 1st vertex
			OBJ_TRIANGLE_VI[0][FIC] = (int)atof(std::strtok(token1, "/")) - 1;	//save the vertex in the vertices
			OBJ_TRIANGLE_TI[0][FIC] = (int)atof(std::strtok(NULL, "/")) - 1;		//save the texture in the textures
			OBJ_TRIANGLE_NI[0][FIC] = (int)atof(std::strtok(NULL, "/")) - 1;		//save the normal in the normals

			// save for 2nd vertex
			OBJ_TRIANGLE_VI[1][FIC] = (int)atof(std::strtok(token2, "/")) - 1;	//save the vertex in the vertices
			OBJ_TRIANGLE_TI[1][FIC] = (int)atof(std::strtok(NULL, "/")) - 1;		//save the texture in the textures
			OBJ_TRIANGLE_NI[1][FIC] = (int)atof(std::strtok(NULL, "/")) - 1;		//save the normal in the normals

			// save for 3rd vertex
			OBJ_TRIANGLE_VI[2][FIC] = (int)atof(std::strtok(token3, "/")) - 1;	//save the vertex in the vertices
			OBJ_TRIANGLE_TI[2][FIC] = (int)atof(std::strtok(NULL, "/")) - 1;		//save the texture in the textures
			OBJ_TRIANGLE_NI[2][FIC] = (int)atof(std::strtok(NULL, "/")) - 1;		//save the normal in the normals

			FIC += 1;
		}
	}
	OBJ_FILE.close();

	std::cout << "Index count during data copying:  V[" << VIC << "], UV[" << UVIC << "], N[" << NIC << "], f[" << FIC << "]" << std::endl;

	//Prepare the data in right format
	totalVertexCount = FIC * 3; // number of F definitions * 3 triangle vertecies
	std::cout << "Total vertex count of complex object: " << totalVertexCount << std::endl;
	vertexList = new float[totalVertexCount * 3];	// number of F definitions * 3 triangle vertecies * 3 values for vertex
	normalList = new float[totalVertexCount * 3];	// number of F definitions * 3 triangle vertecies * 3 values for vertex
	textureList = new float[totalVertexCount * 2];	// number of F definitions * 3 triangle vertecies * 2 values for vertex

	for (int triangle = 0, row; triangle < FIC; triangle++) {
		for (int vertex = 0; vertex < 3; vertex++) {
			// row of the table if each row has the whole triangle
			row = triangle * 9;
			for (int axis = 0; axis < 3; axis++) {
				// load the 3 floats for each of 3 vertexes of each of the triangles
				vertexList[row + axis + (vertex*3)] = OBJ_VERTICES[axis][OBJ_TRIANGLE_VI[vertex][triangle]];
				normalList[row + axis + (vertex*3)] = OBJ_NORMALS[axis][OBJ_TRIANGLE_NI[vertex][triangle]];
			}
			// row of the table if each row has the whole triangle UV
			row = triangle * 6;
			for (int axis = 0; axis < 2; axis++) {
				// load the 2 floats for each of 3 vertexes of each of the triangles
				textureList[row + axis + (vertex * 2)] = OBJ_TEXUV[axis][OBJ_TRIANGLE_TI[vertex][triangle]];
			}
		}
	}

	// free memory in arrays
/*	for (int i = 0; i < 3; i++) {
		if (i < 2) {
			delete[] OBJ_TEXUV[i];
		}
		delete[] OBJ_VERTICES[i];
		delete[] OBJ_NORMALS[i];
		delete[] OBJ_TRIANGLE_VI[i];
		delete[] OBJ_TRIANGLE_NI[i];
		delete[] OBJ_TRIANGLE_TI[i];
	}
	delete[] OBJ_VERTICES;
	delete[] OBJ_NORMALS;
	delete[] OBJ_TEXUV;
	delete[] OBJ_TRIANGLE_VI;
	delete[] OBJ_TRIANGLE_NI;
	delete[] OBJ_TRIANGLE_TI;
*/
	// TO DO
	// adding textures to the mode
	cv::Mat texImage = cv::imread(complexTexFile, CV_LOAD_IMAGE_UNCHANGED);   // Read the file
	glBindTexture(GL_TEXTURE_2D, objectTextureId);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, texImage.cols, texImage.rows, 0, GL_RGBA, GL_UNSIGNED_BYTE, texImage.data);
}

void ARViewer::renderBackgroundGL()
{
	/*
	 * Exercise 1.2 - Show:
	 *                1) A color as background image for the case that capturing frames are missing
	 *                2) capturing frames as background
	 */
	//TODO
	glBindTexture(GL_TEXTURE_2D, textureId);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, current_img.cols, current_img.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, current_img.data);

  	glMatrixMode(GL_MODELVIEW);
  	glLoadIdentity();
  	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glBegin(GL_QUADS);
	glColor3f(1.0f, 1.0f, 1.0f);     // White

   // a quad same size as screen which has normals facing the camera
   // the selected texture is either the loaded frame image or initialized color
   	glTexCoord2f(1.0, 1.0);
	glVertex3f(110.0f,  75.0f, -99.0f);

  	glTexCoord2f(0.0, 1.0);
	glVertex3f(-110.0f,  75.0f, -99.0f);
	
   	glTexCoord2f(0.0, 0.0);
	glVertex3f(-110.0f, -75.0f, -99.0f);
	
   	glTexCoord2f(1.0, 0.0);
	glVertex3f(110.0f, -75.0f, -99.0f);
	glEnd();

	glBindTexture(GL_TEXTURE_2D, 0);

	// after loading we reset the currentimage to capture missing frames
	emptyCurrentImg();

}

