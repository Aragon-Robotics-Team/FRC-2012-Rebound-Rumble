#ifndef _TARGET_H
#define _TARGET_H

#include <vector>
#include "Vision/HSLImage.h"

class Target
{
public:

    static vector<Target> FindRectangularTargets(HSLImage *image);
    
};

#endif
