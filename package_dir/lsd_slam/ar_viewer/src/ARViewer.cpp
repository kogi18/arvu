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
	//complexObjFile = "/home/rok/rosbuild_ws/package_dir/lsd_slam/ar_viewer/assets/nukahedron.obj";
	//complexObjFile = "/home/rok/rosbuild_ws/package_dir/lsd_slam/ar_viewer/assets/vader.obj";
	//complexTexFile = "/home/rok/rosbuild_ws/package_dir/lsd_slam/ar_viewer/assets/vader.png";

	complexObjFile = "/home/rok/rosbuild_ws/package_dir/lsd_slam/ar_viewer/assets/Millennium_Falcon.obj";
	complexTexFile = "/home/rok/rosbuild_ws/package_dir/lsd_slam/ar_viewer/assets/Millennium_Falcon.png";
	
	// complex object animation settings
	/*	PATH
		   0     0
	#	  /	\   / \
	|	 /   \ /   \
	z	0     \     0
	|	 \   / \   /
	#	  \ /   \ /
		   0     0
	       <--x-->
	*/
	int distanceFactor = 3;
	float distanceZ = 6.0f;
	float distanceY = 8.0f;
	float distanceX = distanceZ * sqrt(distanceFactor*distanceFactor - 1); // diagonal is distanceZ * distanceFactor

	circlingMax = 20;
	straightMovementMax = distanceFactor*circlingMax;
	currentFrame = 0;

	circlingRotStepDeg = 180.0f / circlingMax;
	circlingRadius = distanceZ / 2;
	circlingBufferX = distanceX / 2;
	straightSpeedX = distanceX / straightMovementMax;
	straightSpeedZ = distanceZ / straightMovementMax;
	droppingSpeedY = - distanceY / circlingMax;

	CUBE_x = 0.0f;
	CUBE_y = 0.0f;
	CUBE_z = -9.0f;

	CO_x = -distanceX / 2;
	CO_y = 4.0f;
	CO_z = -distanceZ / 2;
	CO_Deg_y = -90.0f;

	isCirgling = 1; // 1 = true
	isFirstHalf = 1;

	mustDrawObjects = 0;
	mustDrawMesh = 1;
	mustDrawGround = 1;
	mustDrawPC = 0;

	emptyCurrentImg();
	if(mustDrawObjects > 0){
		initComplexModel();
	}
}

ARViewer::~ARViewer() {
	current_img.release();
	if(mustDrawPC > 0 || mustDrawMesh > 0){
		delete kfd;
	}
}

void ARViewer::emptyCurrentImg(){
	// set to be green
	current_img = cv::Mat(480,640, CV_8UC3,cv::Scalar(0,127,0));
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
}

void ARViewer::addPoseMsg(geometry_msgs::PoseStampedConstPtr msg){
	boost::lock_guard<boost::mutex> guard(dataMutex); // unlocks the dataMutex at function end
	//std::cout << "POSE TIME: " << msg->header.stamp.sec << std::endl;
	/*
	 * Exercise 1.1: Update camera pose and orientation
	 */
	// we have to negate them
	// it also appears that the given X is positive to the left, so double negate on x
	// for pose vector we have to also scale it from image size to OpenGL size = empirically it seems to be a scale of 10
	qglviewer::Vec pose = qglviewer::Vec(10*msg->pose.position.x, -10*msg->pose.position.y, -10*msg->pose.position.z);
//	qglviewer::Vec pose = qglviewer::Vec(msg->pose.position.x, -msg->pose.position.y, -msg->pose.position.z);
	qglviewer::Quaternion orientation = qglviewer::Quaternion(msg->pose.orientation.x, -msg->pose.orientation.y, -msg->pose.orientation.z, msg->pose.orientation.w);
//	qglviewer::Quaternion orientation = qglviewer::Quaternion(-msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, -msg->pose.orientation.w);



	camera()->setPosition(pose);
	camera()->setOrientation(orientation);

}

void ARViewer::addFrameMsg(ar_viewer::keyframeMsgConstPtr msg){
	if(mustDrawPC > 0 || mustDrawMesh > 0){
		boost::lock_guard<boost::mutex> guard(dataMutex); // unlocks the dataMutex at function end
		//std::cout << "FRAME TIME: " << int(msg->time) << std::endl;
		if(msg->isKeyframe &&  msg->pointcloud.size() > 0){
			std::cout << "ID: " << msg->id << " pointcloud size: " << msg->pointcloud.size() << std::endl;
			if(msg->id > kfd->id){
				kfd->setFrom(msg);
			}
			else{
				//reinitilize the kfd since we have a new input (id < than before)
				// OR SHOULD I RATHER IGNORE IT ?
				std::cout << "MSG ID=" << msg->id << " is smaller than previous saved ID=" << kfd->id << ". Reinitializing KFD." << std::endl;
				delete kfd;
				kfd = new KeyFrameDisplay();
				kfd->mustDrawMesh = mustDrawMesh;
				kfd->setFrom(msg);
			}
		}
	}
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
	  glNormal3f(0.0, 1.0, 0.0); 

	  // Bottom face (y = -1.0f)
	  glColor3f(1.0f, 0.5f, 0.0f);     // Orange
	  glNormal3f(0.0, -1.0, 0.0); 
	  glVertex3f( 1.0f, -1.0f,  1.0f);
	  glVertex3f(-1.0f, -1.0f,  1.0f);
	  glVertex3f(-1.0f, -1.0f, -1.0f);
	  glVertex3f( 1.0f, -1.0f, -1.0f);

	  // Front face  (z = 1.0f)
	  glColor3f(1.0f, 0.0f, 0.0f);     // Red
	  glNormal3f(0.0, 0.0, 1.0); 
	  glVertex3f( 1.0f,  1.0f, 1.0f);
	  glVertex3f(-1.0f,  1.0f, 1.0f);
	  glVertex3f(-1.0f, -1.0f, 1.0f);
	  glVertex3f( 1.0f, -1.0f, 1.0f);

	  // Back face (z = -1.0f)
	  glColor3f(1.0f, 1.0f, 0.0f);     // Yellow
	  glNormal3f(0.0, 0.0, -1.0); 
	  glVertex3f( 1.0f, -1.0f, -1.0f);
	  glVertex3f(-1.0f, -1.0f, -1.0f);
	  glVertex3f(-1.0f,  1.0f, -1.0f);
	  glVertex3f( 1.0f,  1.0f, -1.0f);

	  // Left face (x = -1.0f)
	  glColor3f(0.0f, 0.0f, 1.0f);     // Blue
	  glNormal3f(-1.0, 0.0, 0.0); 
	  glVertex3f(-1.0f,  1.0f,  1.0f);
	  glVertex3f(-1.0f,  1.0f, -1.0f);
	  glVertex3f(-1.0f, -1.0f, -1.0f);
	  glVertex3f(-1.0f, -1.0f,  1.0f);

	  // Right face (x = 1.0f)
	  glColor3f(1.0f, 0.0f, 1.0f);     // Magenta
	  glNormal3f(1.0, 0.0, 0.0); 
	  glVertex3f(1.0f,  1.0f, -1.0f);
	  glVertex3f(1.0f,  1.0f,  1.0f);
	  glVertex3f(1.0f, -1.0f,  1.0f);
	  glVertex3f(1.0f, -1.0f, -1.0f);
   glEnd();  // End of drawing color-cube
   //--------------------------------------------------------------------
}

void ARViewer::drawComplex(){
	// seperation for faster Exercise 1.3 implementation
	glEnable(GL_TEXTURE_2D);
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
	glDisable(GL_TEXTURE_2D);

}

void ARViewer::drawGround(){
	// assumptions: the ground is a straight plain that goes trough the center of the video
	// normal points up - 0 1 0
	// simplify it into a 20x1x20 mesh from z 1 to -1
	int quadsPerAxis = 20, halfPoint = quadsPerAxis / 2;
	float x1, x2, z1, z2;
	glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
	glBegin(GL_QUADS);
		for(int z=0; z<quadsPerAxis-1; z++){
			for(int x=0;x<quadsPerAxis-1;x++){
				x1 = float(x - halfPoint) / halfPoint;
				x2 = float(x + 1 - halfPoint) / halfPoint;
				z1 = float(z - halfPoint) / halfPoint;
				z2 = float(z + 1 - halfPoint) / halfPoint;
				glNormal3f(0.0f, 1.0f, 0.0f); 
				glColor4f(1.0f, 1.0f, 0.0f, 1.0f);
				glVertex3f(x1, 0.0f, z1);
				glVertex3f(x2, 0.0f, z1);
				glVertex3f(x2, 0.0f, z2);
				glVertex3f(x1, 0.0f, z2);
			}
		}
	glEnd();
	glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
}

void ARViewer::draw(){

	glPushMatrix();
	renderBackgroundGL();
	glPopMatrix();

	glScalef(10.0f, 10.0f, 10.0f);
	if(mustDrawGround){
		drawGround();
	}
	if(mustDrawMesh > 0){
		kfd->drawMesh(0.75f);		
	}
	if(mustDrawPC > 0){
		kfd->refreshPC();
		kfd->drawPC();
	}
	glScalef(0.1f, 0.1f, 0.1f);

	/*
	 * Exercise 1.3 - Replace the box with one or more complex 3D objects
	 */

	if(mustDrawObjects){
		glTranslatef(CUBE_x, CUBE_y, CUBE_z);
		drawCube();

		// Animation calculation - 8-circling flight
		currentFrame++;
		if(isCirgling == 1){
			// half circle movement
			if(isFirstHalf == 1){
				// forward direction
				CO_Deg_y += circlingRotStepDeg;
				CO_x = -circlingBufferX -circlingRadius * sin((CO_Deg_y + 90.0f)*deg2rad);
				CO_y += droppingSpeedY;
				CO_z = -circlingRadius * cos((CO_Deg_y + 90.0f)*deg2rad); 
				//DEBUG(circlingRadius * sin((CO_Deg_y + 90.0f)*deg2rad));
			}
			else{
				//returning direction
				CO_Deg_y -= circlingRotStepDeg;
				CO_x = circlingBufferX - circlingRadius * sin((CO_Deg_y - 90.0f)*deg2rad);
				CO_y -= droppingSpeedY;
				CO_z = -circlingRadius * cos((CO_Deg_y - 90.0f)*deg2rad); 
				//DEBUG(circlingRadius * sin((CO_Deg_y - 90.0f)*deg2rad));
			}

			//CO_z += //circlingRadius * 2.0f / circlingMax;

			if(currentFrame >= circlingMax){
				currentFrame = 0;
				isCirgling = 0;
				if(isFirstHalf == 1){
					CO_Deg_y = 90.f;
				}
				else{
					CO_Deg_y = -90.f;				
				}
			}
		}
		else{
			// diagonal movement
			if(isFirstHalf == 1){
				// forward direction
				CO_x += straightSpeedX;
				CO_y += 0.0f;
			}
			else{
				//returning direction
				CO_x -= straightSpeedX;
				CO_y += 0.0f;
			}

			CO_z -= straightSpeedZ;

			if(currentFrame >= straightMovementMax){
				currentFrame = 0;
				isCirgling = 1;
				isFirstHalf = 1 - isFirstHalf;
			}
		}
		//std::cout << "X: " << CO_x + CUBE_x << " Y: " << CO_y + CUBE_y << " Z: " << CO_z + CUBE_z<< std::endl;
		glTranslatef(CO_x , CO_y, CO_z);
		glRotatef(CO_Deg_y, 0.0f, 1.0f, 0.0f); // rotation around Y
		drawComplex();
	}
}

void ARViewer::init(){

	if(mustDrawPC > 0 || mustDrawMesh > 0){
		kfd = new KeyFrameDisplay();
		kfd->mustDrawMesh = mustDrawMesh;
	}
	glGenTextures(1, &textureId);
	glBindTexture(GL_TEXTURE_2D, textureId);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );

	glGenTextures(1, &objectTextureId);
	glBindTexture(GL_TEXTURE_2D, objectTextureId);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );

	// adding textures for the complex model
	if(mustDrawObjects > 0){
		cv::Mat texImage = cv::imread(complexTexFile, CV_LOAD_IMAGE_UNCHANGED);   // Read the file
		glBindTexture(GL_TEXTURE_2D, objectTextureId);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texImage.cols, texImage.rows, 0, GL_RGBA, GL_UNSIGNED_BYTE, texImage.data);
		glBindTexture(GL_TEXTURE_2D, 0);
		texImage.release();
	}

	glBindTexture(GL_TEXTURE_2D, 0);

	setAnimationPeriod(1000/30);
	startAnimation();

	camera()->setZClippingCoefficient(100);
	camera()->setZNearCoefficient(0.0001);
	camera()->setAspectRatio(640.0/480.0);
	camera()->setFieldOfView(0.785398*170.0/90.0*640.0/752.0);//0.785398); //vertical
}

void ARViewer::initComplexModel(){
	//code rearranged from my (Rok's) previous project

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
	for (int i = 0; i < 3; i++) {
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
}

void ARViewer::renderBackgroundGL()
{
	/*
	 * Exercise 1.2 - Show:
	 *                1) A color as background image for the case that capturing frames are missing
	 *                2) capturing frames as background
	 */
	//TODO
  	glMatrixMode(GL_MODELVIEW);
  	glLoadIdentity();

	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, textureId);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, current_img.cols, current_img.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, current_img.data);

	glBegin(GL_QUADS);
	glColor3f(1.0f, 1.0f, 1.0f);     // White

   // a quad same size as screen which has normals facing the camera
   // the selected texture is either the loaded frame image or initialized color
   	glTexCoord2f(1.0, 1.0);
   	glNormal3f(0.0, 0.0, 1.0); 
	glVertex3f(110.0f,  75.0f, -99.0f);

  	glTexCoord2f(0.0, 1.0);
   	glNormal3f(0.0, 0.0, 1.0); 
	glVertex3f(-110.0f,  75.0f, -99.0f);
	
   	glTexCoord2f(0.0, 0.0);
   	glNormal3f(0.0, 0.0, 1.0); 
	glVertex3f(-110.0f, -75.0f, -99.0f);
	
   	glTexCoord2f(1.0, 0.0);
   	glNormal3f(0.0, 0.0, 1.0); 
	glVertex3f(110.0f, -75.0f, -99.0f);
	glEnd();

	glBindTexture(GL_TEXTURE_2D, 0);
	glDisable(GL_TEXTURE_2D);

	// after loading we reset the currentimage to capture missing frames
	emptyCurrentImg();

}

