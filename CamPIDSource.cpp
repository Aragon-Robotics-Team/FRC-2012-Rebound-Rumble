#include "CamPIDSource.h"

/**
 * PID Source from the camera (center x-coordinate of the rectangular target).
 * Outputs range from -1 to 1 (where 0 is the center of the camera).
 */

CamPIDSource::CamPIDSource()
{}

void CamPIDSource::SetSource(double newSource)
{
	source = newSource;
}

double CamPIDSource::PIDGet()
{
	return source;
};
