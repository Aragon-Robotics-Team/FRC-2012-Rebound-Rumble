#include "WPILib.h"
#include <math.h>

/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */ 
class RobotDemo : public SimpleRobot
{
	RobotDrive *drive; // robot drive system
	KinectStick *leftArm;	//The Left arm should be constructed as stick 1
	KinectStick *rightArm; 	//The Right arm should be constructed as stick 2
	Joystick *leftStick, *rightStick; // joysticks
	Jaguar *roller; // ball-collecting roller
	Jaguar *shooter; // shooter wheels
	Gyro *gyro; // gyro
	
	float leftSpeed, rightSpeed;
	float distance;
	double gyroAngle;
	bool rollerOn;

public:
	RobotDemo(void)
	{
		drive = new RobotDrive(1, 2);
		leftArm = new KinectStick(1);
		rightArm = new KinectStick(2);
		leftStick = new Joystick(1);
		rightStick = new Joystick(2);
		//roller = new Jaguar(#);
		//shooter = new Jaguar(#);
		//gyro = new Gyro(5);
		
		// Initialize camera
		AxisCamera &camera = AxisCamera::GetInstance("10.8.40.11");
		camera.WriteResolution(AxisCamera::kResolution_320x240);
		camera.WriteCompression(20);
		camera.WriteBrightness(0);
		
		GetWatchdog().SetExpiration(0.1);
	}
	
	// limits maximum change in acceleration to protect chains
	float SoftStart(float current, float target, float limit = 0.02)
	{
		float result = target;
		float error = current - target;
		if (current * error <= 0)
		{
			if (error > limit)
				result = current - limit;
			if (error < -limit)
				result = current + limit;
		}
		if (result * current < 0)
			result = 0;
		return result;
	}
	
	/*
	// get distance based on camera image
	float getDistance(cameraInputStuff)
	{
		return distance;
	}
	
	// rotates Lazy Susan to face target based on camera image
	void rotateShooter(cameraInputStuff)
	{
		if (leftStick->GetTrigger(1))
		{
			if cameraInputStuff is to the right
				rotate to right
			else if to the left
				rotate to the left
			else
				stop rotating
		} else
			stop rotating
	}*/
	
	// adjusts speed of shooter motor based on distance from target
	void adjustShooter(float distance)
	{
		// adjust shooter RPM based on distance and constant ratio
		float power = sqrt(distance * 26688.017) / 15000;
		if (power > 1.0)
			power = 1.0;
		else if (power < -1.0)
			power = -1.0;
		shooter->Set(power);
	}
	
	// fire shooter
	void launch()
	{
		// fire stuff
	}
	
	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	void Autonomous(void)
	{
		/*A loop is necessary to retrieve the latest Kinect data and update the motors */
		while(IsAutonomous())
		{
			leftSpeed = SoftStart(leftSpeed, leftArm->GetY()*.7);
			rightSpeed = SoftStart(rightSpeed, rightArm->GetY()*.7);
			drive->TankDrive(leftSpeed, rightSpeed);
			
			// aim and ready shooter
			//rotateShooter(cameraInputStuff);
			//adjustShooter(getDistance(cameraInputStuff));
			
			Wait(.01); // Delay 10ms to reduce processing load
		}
		/*drive.Drive(0.5, 0.0); 	// drive forwards half speed
		Wait(2.0); 					// for 2 seconds
		drive.Drive(0.0, 0.0);*/ 	// stop robot
	}
		
	/**
	 * Runs the motors with tank steering. 
	 */
	void OperatorControl(void)
	{
		GetWatchdog().SetEnabled(true);
		AxisCamera &camera = AxisCamera::GetInstance("10.8.40.11");
		DriverStationLCD *ds = DriverStationLCD::GetInstance();
		
		while (IsOperatorControl()) // loop forever
		{
			GetWatchdog().Feed();
			
			// drive with tank drive
			leftSpeed = SoftStart(leftSpeed, leftStick->GetY());
			rightSpeed = SoftStart(rightSpeed, rightStick->GetY());
			drive->TankDrive(leftSpeed, rightSpeed);
			
			
			// turn on roller via joystick buttons
			/*if (leftStick->GetButton(1))
			{
				if (rollerOn == true)
					roller->Set(1.0);
				else
					roller->Set(0.0);
			}*/
			
			//gyroAngle = gyro->GetAngle();
			
			int pixels = 0;
			if (camera.IsFreshImage())
			{
				HSLImage *image = camera.GetImage();
				pixels = image->GetHeight() * image->GetWidth();
				Image *raw_image = image->GetImaqImage();
				PixelValue pv;
				imaqGetPixel(raw_image, imaqMakePoint(0, 0), &pv);
				//for (int i = 0; i < pixels; i++)
				//{
					
				//}
				delete image;
				ds->PrintfLine(DriverStationLCD::kUser_Line2, "pixels: %d", pixels);
				ds->PrintfLine(DriverStationLCD::kUser_Line3, "pixel value: %f", pv.grayscale);
			}
			
			// aim and ready shooter
			//rotateShooter(cameraInputStuff);
			//adjustShooter(getDistance(cameraInputStuff));
			
			// shoot if right trigger pressed
			/*if (rightStick->GetTrigger(1))
				launch();
			else
				noLaunch();
			*/
			
			// print LCD messages
			ds->PrintfLine(DriverStationLCD::kUser_Line1, "angle: %.2f", gyroAngle);
			ds->UpdateLCD();
			
			Wait(0.005);				// wait for a motor update time
		}
	}
};

START_ROBOT_CLASS(RobotDemo);

