#include "Target.h"
#include "nivision.h"
#include <math.h>
#include <algorithm>

/**
 * The Target class contains functions to check for rectangular targets and calculate
 * a number of parameters including height, width, position, and distance.
 */

// Configure constants for thresholding image to get light from red LED
#define HUE_MIN 50
#define HUE_MAX 255
#define SAT_MIN 0
#define SAT_MAX 255
#define LUM_MIN 175
#define LUM_MAX 255
#define EROSION_ITERATIONS 2

// Returns target with greatest score for sorting
bool compareTargetTop(Target t1, Target t2)
{
	double t1Score = (t1.m_areaScore - t1.m_yPos) / 2;
	double t2Score = (t2.m_areaScore - t2.m_yPos) / 2;
	return (t1Score > t2Score);
}
bool compareTargetLeft(Target t1, Target t2)
{
	double t1Score = (t1.m_areaScore - t1.m_xPos) / 2;
	double t2Score = (t2.m_areaScore - t2.m_xPos) / 2;
	return (t1Score > t2Score);
}
bool compareTargetRight(Target t1, Target t2)
{
	double t1Score = (t1.m_areaScore + t1.m_xPos) / 2;
	double t2Score = (t2.m_areaScore + t2.m_xPos) / 2;
	return (t1Score > t2Score);
}
bool compareTargetBottom(Target t1, Target t2)
{
	double t1Score = (t1.m_areaScore + t1.m_yPos) / 2;
	double t2Score = (t2.m_areaScore + t2.m_yPos) / 2;
	return (t1Score > t2Score);
}

char* Target::TargetToString(TargetType targetType)
{
	switch (targetType)
	{
	case kLeftTarget:
		return "left";
	case kRightTarget:
		return "right";
	case kTopTarget:
		return "top";
	case kBottomTarget:
		return "bottom";
	default:
		return "unknown";
	}
}

// Returns the topmost target found in camera vision
Target Target::FindRectangularTarget(HSLImage *image, TargetType targetType = kTopTarget)
{
	ParticleFilterCriteria2 criteria[] = {
								{IMAQ_MT_BOUNDING_RECT_WIDTH, 30, 400, false, false},
								{IMAQ_MT_BOUNDING_RECT_HEIGHT, 40, 400, false, false}
	};
	
	// set threshold based on hue, saturation, luminance
	BinaryImage *binaryImage = image->ThresholdHSL(HUE_MIN, HUE_MAX, SAT_MIN, SAT_MAX, LUM_MIN, LUM_MAX); // get just red target pixels
	BinaryImage *bigObjectsImage = binaryImage->RemoveSmallObjects(false, EROSION_ITERATIONS);  // remove small objects (noise)
	BinaryImage *convexHullImage = bigObjectsImage->ConvexHull(false);  // fill in partial and full rectangles
	BinaryImage *filteredImage = convexHullImage->ParticleFilter(criteria, 2);  // find the rectangles
	vector<ParticleAnalysisReport> *reports = filteredImage->GetOrderedParticleAnalysisReports();  // get the results
	
	/*Image *processedImage = binaryImage->GetImaqImage();
	// fill rectangles with convex hull function
	imaqConvexHull(processedImage, processedImage, true);
	// remove small particles
	imaqSizeFilter(processedImage, processedImage, true, EROSION_ITERATIONS, IMAQ_KEEP_LARGE, NULL);
	
	int particleCount;
	frcCountParticles(processedImage, &particleCount);*/
	
	vector<Target> sortedTargets;
	
	// Fill target vector based on particles
	//for (int i = 0; i < particleCount; i++)
	//{
	for (unsigned i = 0; i < reports->size(); i++) {
		ParticleAnalysisReport *r = &(reports->at(i));
		//ParticleAnalysisReport report;
		//frcParticleAnalysis(processedImage, i, &report);
		Target target;
		target.m_xPos = r->center_mass_x_normalized;
		target.m_yPos = r->center_mass_y_normalized;
		target.m_width = r->boundingRect.width;
		target.m_height = r->boundingRect.height;
		target.m_areaScore = r->particleArea / (target.m_width * target.m_height);
		
		// add results to vector
		//if (target.m_width > 30 && target.m_height > 30) && target.m_areaScore > 0.60)
			sortedTargets.push_back(target);
	}
	
	// delete binary images
	delete reports;
	delete filteredImage;
	delete convexHullImage;
	delete bigObjectsImage;
	delete binaryImage;
	
	switch (targetType)
	{
	case kTopTarget: // sort targets based on score from area and y-coordinate
		sort(sortedTargets.begin(), sortedTargets.end(), compareTargetTop);
		break;
	case kLeftTarget: // sort targets based on score from area and x-coordinate
		sort(sortedTargets.begin(), sortedTargets.end(), compareTargetLeft);
		break;
	case kRightTarget: // sort targets based on score from area and x-coordinate
		sort(sortedTargets.begin(), sortedTargets.end(), compareTargetRight);
		break;
	case kBottomTarget: // sort targets based on score from area and y-coordinate
		sort(sortedTargets.begin(), sortedTargets.end(), compareTargetBottom);
		break;
	}
	
	if (sortedTargets.size() > 0)
		return sortedTargets[0];
	
	Target target;
	target.m_xPos = target.m_yPos = target.m_width = target.m_height = 0;
	return target;
}
