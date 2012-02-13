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
	//Kinect *kinect; // Kinect
	KinectStick *leftArm, *rightArm; // left and right arm for Kinect
	
	Jaguar *leverMotor; // lever motor
	Jaguar *conveyorMotor; // ball-collecting conveyor
	//Victor *rollerMotor; // midpoint roller that prevents shooting
	//Jaguar *turretMotor; // Window motor that powers Lazy Susan
	Jaguar *shooterMotor1; // shooter wheel 1
	Jaguar *shooterMotor2; // shooter wheel 2
	
	DigitalInput *counterSwitch; // rotary switch that detects balls in conveyor
	//Gyro *gyro; // gyro for measuring elevationAngle
	AnalogChannel *turretPot; // potentiometer for measuring turretAngle
	AnalogChannel *leverPot; // potentiometer for measuring leverAngle
	
	CamPIDSource *cameraSource; // PID Source for camera
	PIDController *turretControl; // PID controller for centering target in camera's field
	//PIDController *balanceControl; // PID controller for balancing on bridge with gyro
	Timer *switchTimer; // timer that measures time between counting balls
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
	bool autoAiming; // if camera autonomously aims turret
	bool launching; // if firing ball
	bool balancing; // if autonomous bridge balance enabled

public:
	RobotDemo(void)
	{
		// Initialize drivetrain, Kinect, and joysticks
		//drive = new RobotDrive(1, 2);
		leftStick = new Joystick(1);
		rightStick = new Joystick(2);
		leftArm = new KinectStick(1);
		rightArm = new KinectStick(2);
		
		// Initialize motors
		//leverMotor = new Jaguar(5);
		conveyorMotor = new Jaguar(4);
		//rollerMotor = new Jaguar(2); // change slot
		//turretMotor = new Jaguar(5);
		shooterMotor1 = new Jaguar(6);
		shooterMotor2 = new Jaguar(7);
		
		// Initialize sensors and PID Controller
		counterSwitch = new DigitalInput(1);
		//gyro = new Gyro(1); // change slot
		//gyro->Reset();
		//gyro->SetSensitivity(0.01);
		turretPot = new AnalogChannel(2);
		leverPot = new AnalogChannel(3);
		
		cameraSource = new CamPIDSource();
		//turretControl = new PIDController(0.3, 0.000, 0.0, cameraSource, turretMotor);
		//turretControl->SetOutputRange(-0.8, 0.8);
		//turretControl->Enable();
		//balanceControl = new PIDController(0.1, 0.0, 0.1, gyro, drive);
		//balanceControl->Disable();
		switchTimer = new Timer();
		launchTimer = new Timer();
		
		ballCounter = 0;
		autoAiming = false;
		launching = false;
		balancing = false;
		
		// Initialize camera
		AxisCamera &camera = AxisCamera::GetInstance("10.8.40.11");
		camera.WriteResolution(AxisCamera::kResolution_320x240);
		camera.WriteCompression(30);
		camera.WriteBrightness(30);
		Wait(3.0);

		GetWatchdog().SetExpiration(300);
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
	
	// updates conveyor motor based on number of balls in possession
	void updateConveyor(Timer *timer)
	{
		const int BALL_LIMIT = 3; // max balls in possession
		
		if (!switchPressed && counterSwitch->Get())
		{
			ballCounter++;
			switchPressed = true;
			timer->Start();
		}
		else if (switchPressed && !counterSwitch->Get() && timer->Get() > 3.0)
		{
			switchPressed = false;
			timer->Reset();
		}
		
		// Turn on conveyor if less than 3 balls in possession or shooting
		//if ((ballCounter < BALL_LIMIT || launching) && conveyorMotor->Get() < 1.0)
		if (leftStick->GetTrigger(Joystick::kLeftHand))
			conveyorMotor->Set(1.0);
		else if (conveyorMotor->Get() > 0.0)
			conveyorMotor->Set(0.0);
	}
	
	void rotateLever(float speed)//targetAngle)
	{
		const float MIN_ANGLE = 0.0;
		const float MAX_ANGLE = 150.0;
		
		leverAngle = leverPot->GetValue();
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
		const float MIN_ANGLE = 245;
		const float MAX_ANGLE = 490;
		
		turretAngle = turretPot->GetValue();
		if ((turretAngle <= MIN_ANGLE && position < 0.0) || (turretAngle >= MAX_ANGLE && position > 0.0))
			position = 0.0;
		
		/*if (turretAngle < MIN_ANGLE + 20 && position > 0.0)
			position /= 20 / (turretAngle - MIN_ANGLE);
		else if (turretAngle > MAX_ANGLE - 20 && position < 0.0)
			position /= 20 / (MAX_ANGLE - turretAngle);*/
		
		//if (fabs(position) < 0.1)
		//	position = 0.0;
		
		//cameraSource->SetSource(position);
		//turretControl->SetSetpoint(0);
	}
	
	// adjusts speed of shooter motor based on distance from target; returns speed
	float adjustShooter(float distance, bool override = false)
	{
		float power;
		
		// if joystick override
		if (override)
			power = distance;
		else
		{
			// adjust shooter RPM based on distance and constant ratio
			float power = sqrt(distance * 26688.017) / 15000;
			if (power > 1.0)
				power = 1.0;
			else if (power < -1.0)
				power = -1.0;
		}
		
		if (ballCounter > 0 || override)
		{
			shooterMotor1->Set(power);
			shooterMotor2->Set(power);
		}
		return power;
	}
	
	// fire shooter
	/*void launch()
	{
		// power midpoint roller to get ball from conveyor
		if (ballCounter > 0 && rollerMotor->Get() < 1.0 && launchTimer->Get() == 0.0)
		{
			//rollerMotor->Set(0.5);
			launching = true;
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
		//kinect = Kinect::GetInstance();
		
		/*A loop is necessary to retrieve the latest Kinect data and update the motors */
		while(IsAutonomous())
		{
			leftSpeed = SoftStart(leftSpeed, leftArm->GetY()*.7);
			rightSpeed = SoftStart(rightSpeed, rightArm->GetY()*.7);
			//drive->TankDrive(leftSpeed, rightSpeed);
			
			updateConveyor(switchTimer);
			
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
						rotateTurret(newPosition);
						shooterSpeed = adjustShooter(targetDistance);
					}
				}
			}
			
			// fire ball if Kinect senses head left
			//if (hasGesture(kinect, 1))
			//{
				//launch();
			//}
			
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
			//elevationAngle = gyro->GetAngle();
			if (balancing) // autonomous bridge balancing
			{
				//balanceControl->Enable();
				//balanceControl->SetSetpoint(0);
			}
			else // drive with tank drive
			{
				leftSpeed = SoftStart(leftSpeed, leftStick->GetY());
				rightSpeed = SoftStart(rightSpeed, rightStick->GetY());
				//drive->TankDrive(leftSpeed, rightSpeed);
			}
			
			updateConveyor(switchTimer);
			
			// shoot if both triggers pressed
			//if (launching == false && leftStick->GetTrigger(1) && rightStick->GetTrigger(1))
				//launch();
			
			/*if (launching && launchTimer->Get() > 5.0)
			{
				rollerMotor->Set(0.0);
				launchTimer->Reset();
				launching = false;
			}*/
			
			// rotate lever if Button 2 pressed
			/*if (leftStick->GetButton(Joystick::kTopButton))
				rotateLever(1.0);//leverAngle + 5);
			else if (rightStick->GetButton(Joystick::kTopButton))
				rotateLever(-1.0);//leverAngle - 5);
			else
				leverMotor->Set(0.0);*/
			
			cameraTimer->Reset();
			cameraTimer->Start();
			
			autoAiming = leftStick->GetButton(Joystick::kTopButton);
			
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
						if (autoAiming)
						{
							rotateTurret(newPosition);
							shooterSpeed = adjustShooter(targetDistance);
						}
						else
						{
							rotateTurret(leftStick->GetX());
							shooterSpeed = adjustShooter(leftStick->GetY(), true);
						}
						
					}
				}
			}
			cameraTimer->Stop();
			
			// print LCD messages
			ds->PrintfLine(DriverStationLCD::kUser_Line1, "x pos: %.2f", newPosition);
			ds->PrintfLine(DriverStationLCD::kUser_Line2, "target distance: %.2f", targetDistance);
			ds->PrintfLine(DriverStationLCD::kUser_Line3, "shooter speed: %.2f", shooterSpeed);
			//ds->PrintfLine(DriverStationLCD::kUser_Line4, "gyro angle: %.2f", elevationAngle);
			//ds->PrintfLine(DriverStationLCD::kUser_Line4, "lever angle: %.2f", leverAngle);
			ds->PrintfLine(DriverStationLCD::kUser_Line4, "turret angle: %.2f", turretAngle);
			ds->PrintfLine(DriverStationLCD::kUser_Line5, "vision time: %.5f", cameraTimer->Get());
			ds->PrintfLine(DriverStationLCD::kUser_Line6, "balls: %d", ballCounter);
			ds->UpdateLCD();

			Wait(0.001);				// wait for a motor update time
		}
	}
};

START_ROBOT_CLASS(RobotDemo);

