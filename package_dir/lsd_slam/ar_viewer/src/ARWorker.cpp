/*
 * ARWorker.cpp
 *
 * AR - Exercise
 *
 */

#include "ARWorker.h"

#include "../../lsd_slam_core/src/IOWrapper/Timestamp.h"
#include "../../lsd_slam_core/src/IOWrapper/TimestampedObject.h"
#include "ARViewer.h"
#include "IOWrapper/InputImageStream.h"

#define DEBUG(s) std::cout << __FUNCTION__ << " " << s << std::endl;

ARWorker::ARWorker(ARViewer* viewer,lsd_slam::InputImageStream* imageStream) {
	this->imageStream = imageStream;
	this->viewer = viewer;
	imageStream->getBuffer()->setReceiver(this);
}

ARWorker::~ARWorker() {
}

void ARWorker::Loop(){

	while (true) {
		boost::unique_lock<boost::recursive_mutex> waitLock(imageStream->getBuffer()->getMutex());
		while (!(imageStream->getBuffer()->size() > 0)) {
			notifyCondition.wait(waitLock);
		}
		/*
		 * Exercise 1.2
		 */
		//TODO
		viewer->setImage(imageStream->getBuffer()->popFront().data);
		waitLock.unlock();
	}
}


