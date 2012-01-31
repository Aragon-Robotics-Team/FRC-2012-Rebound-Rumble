#include "WPILib.h"
#include "Target.h"
#include "CamPIDSource.h"
#include <AnalogChannel.h>
#include <math.h>

/**
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
	Victor *conveyorMotor; // ball-collecting conveyor
	Jaguar *turretMotor; // Window motor that powers Lazy Susan
	Jaguar *shooterMotor; // shooter wheels
	Victor *leverMotor; // lever motor
	Gyro *gyro; // gyro for measuring elevationAngle
	AnalogChannel *turretPot; // potentiometer for measuring turretAngle
	
	CamPIDSource *cameraSource; // PID Source for camera
	PIDController *turretControl; // PID controller for centering target in camera's field
	
	float leftSpeed, rightSpeed;
	float distance; // distance from target
	float turretAngle; // angle of turret and Lazy Susan
	float shooterSpeed; // speed of shooterMotor
	float elevationAngle; // angle of robot from ground
	int ballCounter; // number of balls in possession

public:
	RobotDemo(void)
	{
		// Initialize drivetrain, Kinect, and joysticks
		drive = new RobotDrive(1, 2);
		leftArm = new KinectStick(1);
		rightArm = new KinectStick(2);
		leftStick = new Joystick(1);
		rightStick = new Joystick(2);
		
		// Initialize motors
		conveyorMotor = new Victor(1); // change slot
		turretMotor = new Jaguar(1); // change slot
		shooterMotor = new Jaguar(2); // change slot
		leverMotor = new Victor(2); // change slot
		
		// Initialize sensors and PID Controller
		gyro = new Gyro(5); // change slot
		turretPot = new AnalogChannel(1); // change slot
		
		cameraSource = new CamPIDSource();
		turretControl = new PIDController(0.1, 0.001, 0.0, cameraSource, turretMotor);
		turretControl->Enable();
		
		// Initialize camera
		AxisCamera &camera = AxisCamera::GetInstance("10.8.40.11");
		camera.WriteResolution(AxisCamera::kResolution_320x240);
		camera.WriteCompression(30);
		camera.WriteBrightness(0);
		
		GetWatchdog().SetExpiration(10);
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
	void deployLever()
	{
		
	}*/
	
	
	// get distance based on camera image's target width
	float getDistance(float targetWidth)
	{
		return distance;
	}
	
	// rotates turret and Lazy Susan to face target based on camera image
	void rotateTurret(float newPosition)
	{
		// set limit based on turret angle
		turretAngle = turretPot->GetVoltage();
		if (turretAngle <= 0 || turretAngle >= 512) // adjust bounds
			newPosition = 0;
		
		cameraSource->SetSource(newPosition);
		turretControl->SetSetpoint(0);
		
	}
	
	// adjusts speed of shooter motor based on distance from target; returns speed
	float adjustShooter(float distance)
	{
		// adjust shooter RPM based on distance and constant ratio
		float power = sqrt(distance * 26688.017) / 15000;
		if (power > 1.0)
			power = 1.0;
		else if (shooterSpeed < -1.0)
			power = -1.0;
		shooterMotor->Set(power);
		return power;
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
		GetWatchdog().SetEnabled(false);
		/*A loop is necessary to retrieve the latest Kinect data and update the motors */
		while(IsAutonomous())
		{
			leftSpeed = SoftStart(leftSpeed, leftArm->GetY()*.7);
			rightSpeed = SoftStart(rightSpeed, rightArm->GetY()*.7);
			drive->TankDrive(leftSpeed, rightSpeed);
			
			// aim and ready shooter
			//rotateTurret(cameraInputStuff);
			//shooterSpeed = adjustShooter(getDistance(cameraInputStuff));
			
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
			
			// turn on conveyor if balls in possession < 3
			/*if (balls < 3)
				conveyorMotor->Set(1.0);
			else
				conveyorMotor->Set(0.0);
			}*/
			
			//elevationAngle = gyro->GetAngle();
			
			if (camera.IsFreshImage())
			{
				HSLImage *image = camera.GetImage();
				
				// Find FRC targets
				vector<Target> targets = Target::FindRectangularTargets(image);
				delete image;
				
				//float targetWidth = targets.targetWidth;
				//distance = getDistance(targetWidth);
				//float newPosition = targets.center_x;
				
				// aim and ready shooter
				//rotateTurret(newPosition);
				//shooterSpeed = adjustShooter(distance);
			}
			
			// shoot if right trigger pressed
			/*if (rightStick->GetTrigger(1))
				launch();
			else
				noLaunch();
			*/
			
			// print LCD messages
			ds->PrintfLine(DriverStationLCD::kUser_Line1, "angle: %.2f", turretAngle);
			ds->PrintfLine(DriverStationLCD::kUser_Line2, "shooter speed: %.2f", shooterSpeed);
			ds->UpdateLCD();
			
			Wait(0.005);				// wait for a motor update time
		}
	}
};

START_ROBOT_CLASS(RobotDemo);

