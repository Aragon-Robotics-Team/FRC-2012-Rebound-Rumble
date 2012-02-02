#include "Target.h"
#include "nivision.h"
#include <math.h>

/**
 * The Target class contains functions to check for rectangular targets and calculate
 * a number of parameters including height, width, position, and distance.
 */

// Returns the topmost target found in camera vision
Target Target::FindRectangularTarget(HSLImage *image)
{
	BinaryImage *binImage = image->ThresholdHSL(0, 10, 0, 5, 250, 255);
	int particles = binImage->GetNumberParticles();
	ParticleAnalysisReport report;
	vector<Target> sortedTargets;
	
	for (int i = 0; i < particles; i++)
	{
		binImage->GetParticleAnalysisReport(i, &report);
		Target target;
		target.m_xPos = report.center_mass_x_normalized;
		target.m_width = report.boundingRect.width;
		// do stuff
		sortedTargets.push_back(target);
	}
	delete binImage;
	
	if (sortedTargets.size() > 0)
		return sortedTargets[0];
	
	Target target;
	return target;
}
