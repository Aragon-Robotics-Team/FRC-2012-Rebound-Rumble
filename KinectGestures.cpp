#include "WPILib.h"
#include "KinectGestures.h"
#include <math.h>

/**
 * Advanced gestures for the Kinect can be called as follows:
 * 
 */

#define M_PI 3.14159

//Constants which define the valid arm positions
#define ARM_MAX_ANGLE 105
#define ARM_MIN_ANGLE -90
#define Z_PLANE_TOLERANCE 0.3	/* In meters */

//Constants which define the "trigger" angles for the various buttons
#define LEG_FORWARD -110
#define LEG_BACKWARD -80
#define LEG_OUT -75
#define HEAD_LEFT 98
#define HEAD_RIGHT 82
    
/**
 * Function to process a pair of joints into an angle in the XY plane. The joints are projected into
 * the XY plane of the Kinect, then an angle is calculated using one joint as the origin and the
 * X-axis as the reference. The X-axis can optionally be mirrored in order to avoid the discontinuity
 * of the atan2 function.
 * @param origin The joint to use as the origin for the calculation
 * @param measured The second point to use as for the angle calculation
 * @param mirrored Whether to mirror the X-axis or not
 * @return Angle in degrees referenced from the X-axis
 */
double AngleXY(Skeleton::Joint origin, Skeleton::Joint measured, UINT8 mirrored)
{
    return (atan2((measured.y - origin.y), (mirrored) ? (origin.x - measured.x) : (measured.x - origin.x))*180/M_PI);
}

/**
 * Funtion to process a pair of joints into an angle in the YZ plane. The joint are projected into
 * the YZ plane of the Kinect, than an angle is calculated using one joint as the origin and the
 * Z-axis as the reference.The Z-axis can optionally be mirrored in order to avoid the discontinuity
 * of the atan2 function.
 * @param origin The joint to use as the origin for the angle calculation
 * @param measured The second point to use as for the angle calculation
 * @param mirrored Whether to mirror the Z-axis or not
 * @return Angle in degrees referenced from the Z-axis
 */
double AngleYZ(Skeleton::Joint origin, Skeleton::Joint measured, UINT8 mirrored)
{
    return (atan2((measured.y - origin.y), (mirrored) ? (origin.z - measured.z) : (measured.z - origin.z))*180/PI);
}

/**
 * Function to determine whether or not two joints are in approximately the same XY plane
 * IE. If they have approximately the same z coordinates
 * @param origin The first joint to be used in the comparison
 * @param measured The second joint to be used in the comparison
 * @return Whether or not the joints are in approximately the same XY plane
 */
bool InSameZPlane(Skeleton::Joint origin, Skeleton::Joint measured, double tolerance)
{
	return fabs(measured.z - origin.z) < tolerance;
}

/**
 * Converts an input value in the given input range into an output value along the given output range.
 * If the result would be outside of the given output range, it is constrained to the output range.
 * @param input An input value within the given input range.
 * @param inputMin The minimum expected input value.
 * @param inputMax The maximum expected input value.
 * @param outputMin The minimum expected output value.
 * @param outputMax The maximum expected output value.
 * @return An output value within the given output range proportional to the input.
 */
double CoerceToRange(double input, double inputMin, double inputMax, double outputMin, double outputMax)
{
	// Determine the center of the input range
	double inputCenter = fabs(inputMax - inputMin) / 2 + inputMin;
	double outputCenter = fabs(outputMax - outputMin) / 2 + outputMin;
	
	// Scale the input range to the output range
	double scale = (outputMax - outputMin) / (inputMax - inputMin);
	
	// Apply the transformation
	double result = (input + -inputCenter) * scale + outputCenter;
	
	// Constrain to the result range
	return max(min(result, outputMax), outputMin);
}

// Check if advanced gesture is active
bool hasGesture(Kinect *kinect, int gesture)
{
    bool dataWithinExpectedRange;
    
    /* Only process data if skeleton is tracked */
    if (kinect->GetTrackingState() == Kinect::kTracked)
    {
    	
    	/* Determine angle of each arm and map to range -1,1 */
    	leftAngle = AngleXY(kinect->GetSkeleton().GetShoulderLeft(), kinect->GetSkeleton().GetWristLeft(), true);
    	rightAngle = AngleXY(kinect->GetSkeleton().GetShoulderRight(), kinect->GetSkeleton().GetWristRight(), false);
    	leftAxis = CoerceToRange(leftAngle, -70, 70, -1, 1);
    	rightAxis = CoerceToRange(rightAngle, -70, 70, -1, 1);
    	
    	/* Check if arms are within valid range and at approximately the same z-value */
    	dataWithinExpectedRange = (leftAngle < ARM_MAX_ANGLE) && (leftAngle > ARM_MIN_ANGLE)
								&& (rightAngle < ARM_MAX_ANGLE) && (rightAngle > ARM_MIN_ANGLE);
    	dataWithinExpectedRange = dataWithinExpectedRange &&
								  InSameZPlane(kinect->GetSkeleton().GetShoulderLeft(),
											   kinect->GetSkeleton().GetWristLeft(),
											   Z_PLANE_TOLERANCE) &&
								  InSameZPlane(kinect->GetSkeleton().GetShoulderRight(),
											   kinect->GetSkeleton().GetWristRight(),
											   Z_PLANE_TOLERANCE);
    	
    	if (dataWithinExpectedRange)
    	{
    		switch (gesture)
    		{
    		case 1:
    			/* Determine the head angle and use it to set the Head buttons */
    			headAngle = AngleXY(kinect->GetSkeleton().GetShoulderCenter(), kinect->GetSkeleton().GetHead(), false);
    			return headAngle > HEAD_LEFT;
    			break;
    			
    		case 2:
    			/* Determine the head angle and use it to set the Head buttons */
    			headAngle = AngleXY(kinect->GetSkeleton().GetShoulderCenter(), kinect->GetSkeleton().GetHead(), false);
    			return headAngle < HEAD_RIGHT;
    			break;
    			
    		case 3:
    			/* Calculate the leg angles in the XY plane and use them to set the Leg Out buttons */
    			leftLegAngle = AngleXY(kinect->GetSkeleton().GetHipLeft(), kinect->GetSkeleton().GetAnkleLeft(), true);
    			return leftLegAngle > LEG_OUT;
    			break;
    			
    		case 4:
    			/* Calculate the leg angles in the XY plane and use them to set the Leg Out buttons */
    			rightLegAngle = AngleXY(kinect->GetSkeleton().GetHipRight(), kinect->GetSkeleton().GetAnkleRight(), false);
    			return rightLegAngle > LEG_OUT;
    			break;
    			
    		case 5:
    			/* Calculate the leg angle in the YZ plane and use them to set the Leg Forward and Leg Back buttons */
    			leftLegYZ = AngleYZ(kinect->GetSkeleton().GetHipLeft(), kinect->GetSkeleton().GetAnkleLeft(), false);
    			return leftLegYZ < LEG_FORWARD;
    			break;
    			
    		case 6:
    			/* Calculate the leg angle in the YZ plane and use them to set the Leg Forward and Leg Back buttons */
    			leftLegYZ = AngleYZ(kinect->GetSkeleton().GetHipLeft(), kinect->GetSkeleton().GetAnkleLeft(), false);
    			return leftLegYZ > LEG_BACKWARD;
    			break;
    			
    		case 7:
    			/* Calculate the leg angle in the YZ plane and use them to set the Leg Forward and Leg Back buttons */
    			rightLegYZ = AngleYZ(kinect->GetSkeleton().GetHipRight(), kinect->GetSkeleton().GetAnkleRight(), false);
    			return rightLegYZ < LEG_FORWARD;
    			break;
    			
    		case 8:
    		    /* Calculate the leg angle in the YZ plane and use them to set the Leg Forward and Leg Back buttons */
    		    rightLegYZ = AngleYZ(kinect->GetSkeleton().GetHipRight(), kinect->GetSkeleton().GetAnkleRight(), false);
    		    return rightLegYZ > LEG_BACKWARD;
    		    break;
    		    
    		default:
    			return false;
    		}
    	}
    }
    
    return false;
}