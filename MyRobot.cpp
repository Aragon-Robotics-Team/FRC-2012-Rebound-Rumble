#include "WPILib.h"
#include "Target.h"
#include "CamPIDSource.h"
#include "KinectGestures.h"
#include <AnalogChannel.h>
#include <math.h>

#define M_PI 3.14159

/**
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */ 
class RobotDemo : public SimpleRobot
{
	RobotDrive *drive; // robot drive system
	Joystick *leftStick, *rightStick; // joysticks
	Kinect *kinect; // Kinect
	KinectStick *leftArm, *rightArm; // left and right arm for Kinect
	Victor *conveyorMotor; // ball-collecting conveyor
	Victor *rollerMotor; // midpoint roller that prevents shooting
	Jaguar *turretMotor; // Window motor that powers Lazy Susan
	Jaguar *shooterMotor; // shooter wheels
	Jaguar *leverMotor; // lever motor
	DigitalInput *counterSwitch; // rotary switch that detects balls in conveyor
	Gyro *gyro; // gyro for measuring elevationAngle
	AnalogChannel *turretPot; // potentiometer for measuring turretAngle
	AnalogChannel *leverPot; // potentiometer for measuring leverAngle
	
	CamPIDSource *cameraSource; // PID Source for camera
	PIDController *turretControl; // PID controller for centering target in camera's field
	PIDController *balanceControl; // PID controller for balancing on bridge with gyro
	Timer *launchTimer; // timer that measures time between firing balls
	
	float leftSpeed, rightSpeed;
	float targetDistance; // distance from target in ft
	float newPosition; // target position from center (-1.0 to 1.0)
	float targetWidth; // width in pixels of target
	float turretAngle; // angle of turret and Lazy Susan
	float shooterSpeed; // speed of shooterMotor
	float leverAngle; // angle of lever
	float elevationAngle; // angle of robot from ground
	int ballCounter; // number of balls in possession
	bool switchPressed; // if limit switch (ballCounter) is pressed
	bool balancing; // if autonomous bridge balance enabled

public:
	RobotDemo(void)
	{
		// Initialize drivetrain, Kinect, and joysticks
		drive = new RobotDrive(1, 2);
		leftStick = new Joystick(1);
		rightStick = new Joystick(2);
		leftArm = new KinectStick(1);
		rightArm = new KinectStick(2);
		
		// Initialize motors
		//conveyorMotor = new Victor(1); // change slot
		//rollerMotor = new Victor(2); // change slot
		//turretMotor = new Jaguar(1); // change slot
		//shooterMotor = new Jaguar(2); // change slot
		leverMotor = new Jaguar(1,5); // change slot
		
		// Initialize sensors and PID Controller
		counterSwitch = new DigitalInput(1); // change slot
		//gyro = new Gyro(1); // change slot
		//gyro->SetSensitivity(0.01);
		turretPot = new AnalogChannel(1); // change slot
		//leverPot = new AnalogChannel(2); // change slot
		
		//cameraSource = new CamPIDSource();
		//turretControl = new PIDController(0.1, 0.001, 0.0, cameraSource, turretMotor);
		//turretControl->SetOutputRange(-0.8, 0.8);
		//turretControl->Enable();
		//bridgeControl = new PIDController(0.1, 0.0, 0.1, gyro, drive);
		//bridgeControl->Disable();
		//launchTimer = new Timer();
		
		ballCounter = 0;
		balancing = false;
		
		// Initialize camera
		AxisCamera &camera = AxisCamera::GetInstance("10.8.40.11");
		camera.WriteResolution(AxisCamera::kResolution_320x240);
		camera.WriteCompression(30);
		camera.WriteBrightness(30);
		Wait(3.0);

		GetWatchdog().SetExpiration(200);
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
	
	// returns angle from potentiometer reading
	float voltageToAngle(float voltage)
	{
		return 0;
	}
	
	// updates conveyor motor based on number of balls in possession
	void updateConveyor()
	{
		const int BALL_LIMIT = 3; // max balls in possession
		
		if (!switchPressed && counterSwitch->Get())
		{
			ballCounter++;
			switchPressed = true;
		}
		else if (switchPressed && !counterSwitch->Get())
		{
			switchPressed = false;
		}
		
		// Turn on conveyor if less than 3 balls in possession
		/*if (ballCounter < BALL_LIMIT) && conveyorMotor->Get() < 1.0)
			conveyorMotor->Set(1.0);
		else if (conveyorMotor->Get() > 0.0)
			conveyorMotor->Set(0.0);*/
	}
	
	void rotateLever(float speed)
	{
		const float MIN_ANGLE = 0.0;
		const float MAX_ANGLE = 90.0;
		
		leverMotor->Set(speed);
		/*if (targetAngle > leverAngle && targetAngle < MAX_ANGLE)
			leverMotor->Set(0.5);
		else if (targetAngle < leverAngle && targetAngle > MIN_ANGLE)
			leverMotor->Set(-0.5);
		else
			leverMotor->Set(0.0);*/
	}
	
	// get distance to target based on camera image's target width in pixels
	float getDistance(float targetWidth)
	{
		float distance = 320.0 / (targetWidth * tan(21.75 * M_PI / 180)); // calibrate later
		return distance;
	}
	
	// rotates turret and Lazy Susan to face target based on camera image
	void rotateTurret(float position)
	{
		// set limit based on turret angle
		//turretAngle = turretPot->GetVoltage();
		//if (turretAngle <= 0 || turretAngle >= 512) // adjust bounds
		//	position = 0.0;
		
		cameraSource->SetSource(position);
		turretControl->SetSetpoint(0);
	}
	
	// adjusts speed of shooter motor based on distance from target; returns speed
	float adjustShooter(float distance)
	{
		// adjust shooter RPM based on distance and constant ratio
		float power = sqrt(distance * 26688.017) / 15000;
		if (power > 1.0)
			power = 1.0;
		else if (power < -1.0)
			power = -1.0;
		//shooterMotor->Set(power);
		return power;
	}
	
	// fire shooter
	/*void launch()
	{
		// power midpoint roller to get ball from conveyor
		if (ballCounter > 0 && rollerMotor->Get() < 1.0 && launchTimer->Get() == 0.0)
		{
			//rollerMotor->Set(0.5);
			launchTimer->Start();
			ballCounter--;
		}
	}*/
	
	/**
	 * Drive motors with tank steering based on Kinect.
	 */
	void Autonomous(void)
	{
		GetWatchdog().SetEnabled(false);
		AxisCamera &camera = AxisCamera::GetInstance("10.8.40.11");
		kinect = Kinect::GetInstance();
		
		/*A loop is necessary to retrieve the latest Kinect data and update the motors */
		while(IsAutonomous())
		{
			leftSpeed = SoftStart(leftSpeed, leftArm->GetY()*.7);
			rightSpeed = SoftStart(rightSpeed, rightArm->GetY()*.7);
			//drive->TankDrive(leftSpeed, rightSpeed);
			
			updateConveyor();
			
			if (camera.IsFreshImage())
			{
				HSLImage *image = camera.GetImage();
				
				// check if image exists
				if (image->GetWidth() && image->GetHeight())
				{
					// Find topmost FRC target
					Target target = Target::FindRectangularTarget(image);
					delete image;
				
					if (target.m_width > 0 && target.m_height > 0)
					{
						newPosition = target.m_xPos;
						targetWidth = target.m_width;
						targetDistance = getDistance(targetWidth);
						
						// aim and ready shooter
						//rotateTurret(newPosition);
						shooterSpeed = adjustShooter(targetDistance);
					}
				}
			}
			
			// fire ball if Kinect senses head left
			if (hasGesture(kinect, 1))
			{
				//launch();
			}
			
			Wait(.01); // Delay 10ms to reduce processing load
		}
	}
		
	/**
	 * Runs the motors with tank steering. 
	 */
	void OperatorControl(void)
	{
		GetWatchdog().SetEnabled(true);
		AxisCamera &camera = AxisCamera::GetInstance("10.8.40.11");
		DriverStationLCD *ds = DriverStationLCD::GetInstance();
		Timer *cameraTimer = new Timer();
		
		while (IsOperatorControl()) // loop forever
		{
			GetWatchdog().Feed();
			//balancing = leftStick->GetTrigger(kLeftHand);
			if (balancing) // autonomous bridge balancing
			{
				//elevationAngle = gyro->GetAngle();
				//bridgeControl->Enable();
				//bridgeControl->SetSetpoint(0);
				// PD output to drive
			}
			else // drive with tank drive
			{
				leftSpeed = SoftStart(leftSpeed, leftStick->GetY());
				rightSpeed = SoftStart(rightSpeed, rightStick->GetY());
				drive->TankDrive(leftSpeed, rightSpeed);
			}
			
			updateConveyor();
			
			// shoot if both triggers pressed
			//if (leftStick->GetTrigger(1) && rightStick->GetTrigger(1))
				//launch();
			
			/*if (launchTimer->Get() > 5.0)
			{
				rollerMotor->Set(0.0);
				launchTimer->Reset();
			}*/
			
			// rotate lever if Button 2 pressed
			//leverAngle = leverPot->GetVoltage();
			if (leftStick->GetButton(Joystick::kTopButton))
				rotateLever(0.5);//leverAngle + 5);
			else if (rightStick->GetButton(Joystick::kTopButton))
				rotateLever(-0.5);//leverAngle - 5);
			else
				leverMotor->Set(0.0);
			
			cameraTimer->Reset();
			cameraTimer->Start();
			
			if (camera.IsFreshImage())
			{
				HSLImage *image = camera.GetImage();
				
				// check if image exists
				if (image->GetWidth() && image->GetHeight())
				{
					// Find topmost FRC target
					Target target = Target::FindRectangularTarget(image);
					delete image;
					
					if (target.m_width > 0 && target.m_height > 0)
					{
						newPosition = target.m_xPos;
						targetWidth = target.m_width;
						targetDistance = getDistance(targetWidth);
						
						// aim and ready shooter
						//rotateTurret(newPosition);
						shooterSpeed = adjustShooter(targetDistance);
					}
				}
			}
			cameraTimer->Stop();
			
			turretAngle = turretPot->GetValue(); // for testing
			
			// print LCD messages
			ds->PrintfLine(DriverStationLCD::kUser_Line1, "turret pot: %.2f", turretAngle);
			//ds->PrintfLine(DriverStationLCD::kUser_Line1, "x pos: %.2f", newPosition);
			ds->PrintfLine(DriverStationLCD::kUser_Line2, "target distance: %.2f", targetDistance);
			ds->PrintfLine(DriverStationLCD::kUser_Line3, "shooter speed: %.2f", shooterSpeed);
			//ds->PrintfLine(DriverStationLCD::kUser_Line4, "gyro angle: %.2f", elevationAngle);
			ds->PrintfLine(DriverStationLCD::kUser_Line4, "lever angle: %.2f", leverAngle);
			ds->PrintfLine(DriverStationLCD::kUser_Line5, "vision time: %.5f", cameraTimer->Get());
			ds->PrintfLine(DriverStationLCD::kUser_Line6, "balls: %d", ballCounter);
			ds->UpdateLCD();

			Wait(0.001);				// wait for a motor update time
		}
	}
};

START_ROBOT_CLASS(RobotDemo);

