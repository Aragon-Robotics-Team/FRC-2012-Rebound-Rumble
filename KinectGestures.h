#ifndef _KINECTGESTURES_H
#define _KINECTGESTURES_H

#include <math.h>

double leftAxis, rightAxis;
double leftAngle, rightAngle, headAngle, rightLegAngle, leftLegAngle, rightLegYZ, leftLegYZ = 0;

bool hasGesture(Kinect *kinect, int gesture);

#endif
