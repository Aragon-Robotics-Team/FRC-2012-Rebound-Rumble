#ifndef _TARGET_H
#define _TARGET_H

#include <vector>
#include "Vision/HSLImage.h"

class Target
{
public:
	double m_xPos;
	double m_width;
	double m_areaScore;
	
    static Target FindRectangularTarget(HSLImage *image);
    
};

#endif
