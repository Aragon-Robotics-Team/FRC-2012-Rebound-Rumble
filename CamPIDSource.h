#include <WPILib.h>

class CamPIDSource : public PIDSource
{
	
public:
	
	CamPIDSource();
	double PIDGet();
	void SetSource(double);
	
private:
	
	double source;
	
};
