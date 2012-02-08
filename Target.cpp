#include "Target.h"
#include "nivision.h"
#include <math.h>
#include <algorithm>

/**
 * The Target class contains functions to check for rectangular targets and calculate
 * a number of parameters including height, width, position, and distance.
 */

// Configure constants for thresholding image to get light from red LED
#define HUE_MIN 200
#define HUE_MAX 255
#define SAT_MIN 0
#define SAT_MAX 255
#define LUM_MIN 150
#define LUM_MAX 255
#define EROSION_ITERATIONS 3

// Returns target with greatest score for sorting
bool compareTargets(Target t1, Target t2)
{
  double t1Score = (t1.m_areaScore + t1.m_yPos) / 2;
  double t2Score = (t2.m_areaScore + t2.m_yPos) / 2;
	return (t1Score < t2Score);
}

// Returns the topmost target found in camera vision
Target Target::FindRectangularTarget(HSLImage *image)
{
	// set threshold based on hue, saturation, luminance
	BinaryImage *binaryImage = image->ThresholdHSL(HUE_MIN, HUE_MAX, SAT_MIN, SAT_MAX, LUM_MIN, LUM_MAX);
	Image *processedImage = binaryImage->GetImaqImage();
	// fill rectangles with convex hull function
	imaqConvexHull(processedImage, processedImage, true);
	// remove small particles
	imaqSizeFilter(processedImage, processedImage, true, EROSION_ITERATIONS, IMAQ_KEEP_LARGE, NULL);
	
	int particleCount;
	frcCountParticles(processedImage, &particleCount);
	
	vector<Target> sortedTargets;
	
	// Fill target vector based on particles
	for (int i = 0; i < particleCount; i++)
	{
		ParticleAnalysisReport report;
		frcParticleAnalysis(processedImage, i, &report);
		Target target;
		target.m_xPos = report.center_mass_x_normalized;
		target.m_yPos = report.center_mass_y_normalized;
		target.m_width = report.boundingRect.width;
		target.m_height = report.boundingRect.height;
		target.m_areaScore = report.particleArea / (target.m_width * target.m_height);
		
		// add to vector if area score is reasonable (resembles rectangle)
		if (target.m_width > 30 && target.m_height > 30 && target.m_areaScore > 0.6)
			sortedTargets.push_back(target);
	}
	
	// delete binary and imaq images
	delete binaryImage;
	imaqDispose(processedImage);
	
	// sort targets based on score from area and y-coordinate
	sort(sortedTargets.begin(), sortedTargets.end(), compareTargets);
	
	if (sortedTargets.size() > 0)
		return sortedTargets[0];
	
	Target target;
	return target;
}
