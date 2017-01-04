/*
 * ARWorker.h
 *
 * AR - Exercise
 *
 */

#ifndef ARWORKER_H_
#define ARWORKER_H_

#include "IOWrapper/NotifyBuffer.h"
#include "IOWrapper/TimestampedObject.h"
#include "geometry_msgs/PoseStamped.h"
#include "IOWrapper/InputImageStream.h"
#include "ar_viewer/keyframeMsg.h"

class ARViewer;
using lsd_slam::TimestampedMat;

class ARWorker: public lsd_slam::Notifiable {
public:
	ARWorker(ARViewer* viewer,lsd_slam::InputImageStream* imageStream);
	virtual ~ARWorker();
	void Loop();
private:
	lsd_slam::InputImageStream* imageStream;
	ARViewer* viewer;
};

//keyframeMsg_()
//: id(0)
//, time(0.0)
//, isKeyframe(false)
//, camToWorld()
//, fx(0.0)
//, fy(0.0)
//, cx(0.0)
//, cy(0.0)
//, height(0)
//, width(0)
//, pointcloud()
//{
//  camToWorld.assign(0.0);
//}

#endif /* ARWORKER_H_ */
