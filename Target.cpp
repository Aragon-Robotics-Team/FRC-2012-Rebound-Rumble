#include "Target.h"
#include "nivision.h"
#include <math.h>
#include <algorithm>

/**
 * The Target class contains functions to check for rectangular targets and calculate
 * a number of parameters including height, width, position, and distance.
 */

// Constants for thresholding image to get light from red ringlight
#define HUE_MIN 0
#define HUE_MAX 255
#define SAT_MIN 0
#define SAT_MAX 255
#define LUM_MIN 200
#define LUM_MAX 255
#define EROSION_ITERATIONS 3

bool compareTargets(Target t1, Target t2)
{
	return (t1.m_areaScore < t2.m_areaScore);
}

// Returns the topmost target found in camera vision
Target Target::FindRectangularTarget(HSLImage *image)
{
	// set threshold based on hue, saturation, luminance
	BinaryImage *image_proc1 = image->ThresholdHSL(HUE_MIN, HUE_MAX, SAT_MIN, SAT_MAX, LUM_MIN, LUM_MAX);
	Image *image_proc2 = imaqCreateImage(IMAQ_IMAGE_U8, 3);
	// fill rectangles with convex hull function
	imaqConvexHull(image_proc2, image_proc1->GetImaqImage(), true);
	Image *image_proc3 = imaqCreateImage(IMAQ_IMAGE_U8, 3);
	// remove small particles
	imaqSizeFilter(image_proc3, image_proc2, true, EROSION_ITERATIONS, IMAQ_KEEP_LARGE, NULL);
	
	int particles;
	frcCountParticles(image_proc3, &particles);
	
	vector<Target> sortedTargets;
	
	// Fill target vector based on particles
	for (int i = 0; i < particles; i++)
	{
		ParticleAnalysisReport report;
		frcParticleAnalysis(image_proc3, i, &report);
		Target target;
		target.m_xPos = report.center_mass_x_normalized;
		target.m_width = report.boundingRect.width;
		target.m_areaScore = report.particleArea / (report.boundingRect.width * report.boundingRect.height);
		
		// add to vector if rectangular enough
		if (target.m_areaScore > 0.5 && target.m_areaScore < 2.0)
			sortedTargets.push_back(target);
	}
	
	// delete binary and imaq images
	delete image_proc1;
	imaqDispose(image_proc2);
	imaqDispose(image_proc3);
	
	// sort targets based on area score
	sort(sortedTargets.begin(), sortedTargets.end(), compareTargets);
	
	if (sortedTargets.size() > 0)
		return sortedTargets[0];
	
	Target target;
	return target;
}
