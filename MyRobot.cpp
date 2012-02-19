#include "WPILib.h"
#include "Gamepad.h"
#include "Target.h"
#include "CamPIDSource.h"
#include "KinectGestures.h"
#include <AnalogChannel.h>
#include <math.h>

#define M_PI 3.14159

#define FRONT_LEFT 1
#define REAR_LEFT 2
#define FRONT_RIGHT 3
#define REAR_RIGHT 4

/**
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */ 
class RobotDemo : public SimpleRobot
{
	RobotDrive *drive; // robot drive system
	Gamepad *drivePad, *turretPad; // gamepads
	//Kinect *kinect; // Kinect
	KinectStick *leftArm, *rightArm; // left and right arm for Kinect
	
	Victor *conveyorMotor; // ball-collecting conveyor
	Jaguar *shooterMotor1; // shooter wheel 1
	Jaguar *shooterMotor2; // shooter wheel 2
	Victor *leverMotor; // lever motor
	Victor *turretMotor; // Window motor that powers Lazy Susan
	
	DigitalInput *entranceSwitch; // rotary switch that detects balls in conveyor
	DigitalInput *exitSwitch; // limit switch that detects balls exiting shooter
	AnalogChannel *turretPot; // potentiometer for measuring turretAngle
	AnalogChannel *leverPot; // potentiometer for measuring leverAngle
	//Gyro *gyro; // gyro for measuring elevationAngle
	
	CamPIDSource *cameraSource; // PID Source for camera
	PIDController *turretControl; // PID controller for centering target in camera's field
	//PIDController *balanceControl; // PID controller for balancing on bridge with gyro
	Timer *switchTimer1; // timer that measures time between counting incoming balls
	Timer *switchTimer2; // timer that measures time between counting exiting balls
	
	float leftSpeed, rightSpeed; // speed of left and right drive motors
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
	
	Target::TargetType targetType; // which target to aim for

public:
	RobotDemo(void)
	{
		// Initialize drivetrain motors
		drive = new RobotDrive(FRONT_LEFT, REAR_LEFT, FRONT_RIGHT, REAR_RIGHT);
		
		// Initialize gamepads and Kinect
		drivePad = new Gamepad(1);
		turretPad = new Gamepad(2);
		leftArm = new KinectStick(1);
		rightArm = new KinectStick(2);
		
		// Initialize motors
		conveyorMotor = new Victor(5); // reversed
		shooterMotor1 = new Jaguar(7);
		shooterMotor2 = new Jaguar(8);
		leverMotor = new Victor(9);
		turretMotor = new Victor(10); // reversed
		
		// Initialize sensors and PID Controller
		entranceSwitch = new DigitalInput(1);
		exitSwitch = new DigitalInput(2);
		turretPot = new AnalogChannel(2);
		leverPot = new AnalogChannel(3);
		//gyro = new Gyro(1); // change slot
		//gyro->Reset();
		//gyro->SetSensitivity(0.01);
		
		cameraSource = new CamPIDSource();
		turretControl = new PIDController(0.3, 0.000, 0.0, cameraSource, turretMotor);
		turretControl->SetOutputRange(-0.8, 0.8);
		turretControl->Enable();
		//balanceControl = new PIDController(0.1, 0.0, 0.1, gyro, drive);
		//balanceControl->Disable();
		switchTimer1 = new Timer();
		switchTimer2 = new Timer();
		
		ballCounter = 0;
		autoAiming = false;
		launching = false;
		balancing = false;
		
		targetType = Target::kTopTarget;
		
		// Initialize camera
		AxisCamera &camera = AxisCamera::GetInstance("10.8.40.11");
		camera.WriteResolution(AxisCamera::kResolution_320x240);
		camera.WriteCompression(30);
		camera.WriteBrightness(30);
		Wait(3.0);

		GetWatchdog().SetEnabled(false);
		drive->SetExpiration(1.0);
	}
	
	// limits maximum change in acceleration to protect chains
	float SoftStart(float current, float target, float limit = 0.01)
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
	void updateConveyor(Timer *timer1, Timer *timer2)
	{
		const int BALL_LIMIT = 3; // max balls in possession
		
		// update ball counter (based on rotary and limit switches)
		if (!switchPressed && entranceSwitch->Get())
		{
			ballCounter++;
			switchPressed = true;
			timer1->Start();
		}
		else if (switchPressed && !entranceSwitch->Get() && timer1->Get() > 3.0)
		{
			switchPressed = false;
			timer1->Reset();
		}
		if (exitSwitch->Get())
		{
			ballCounter--;
			timer2->Start();
		}
		else if (!exitSwitch->Get() && timer2->Get() > 3.0)
		{
			timer2->Reset();
		}
		
		// Turn on conveyor if less than 3 balls in possession, unless shooting
		if (ballCounter == 0 || launching)
			conveyorMotor->Set(-1.0);
		else
			conveyorMotor->Set(0.0);
	}
	
	void updateLever(bool rotateDown = false, bool rotateUp = false)
	{
		const float MIN_ANGLE = 150.0;
		const float MAX_ANGLE = 360.0;
		
		leverAngle = leverPot->GetValue();
		// rotate lever if Button 2 pressed
		if (rotateDown && leverAngle < MAX_ANGLE)
			leverMotor->Set(0.5);
		else if (rotateUp && leverAngle > MIN_ANGLE)
			leverMotor->Set(-0.5);
		else
			leverMotor->Set(0.0);
	}
	
	// get distance to target based on camera image's target width in pixels
	float getDistance(float targetWidth)
	{
		float distance = 320.0 / (targetWidth * tan(12.75 * M_PI / 180)); // calibrate later
		return distance;
	}
	
	// rotates turret and Lazy Susan to face target based on camera image
	void rotateTurret(float position)
	{
		// set limit based on turret angle
		const float MIN_ANGLE = 380;
		const float MAX_ANGLE = 620;
		
		turretAngle = turretPot->GetValue();
		if ((turretAngle <= MIN_ANGLE && position < 0.0) || (turretAngle >= MAX_ANGLE && position > 0.0))
			position = 0.0;
		
		if (fabs(position) <= 0.01)
			position = 0.0;
		
		cameraSource->SetSource(-position);
		turretControl->SetSetpoint(0);
	}
	
	// returns speed of shooter motor based on distance from target
	float getSpeed(float distance, bool override = false)
	{
		float power;
		
		// if joystick override
		if (override)
			power = distance;
		else
		{
			// adjust shooter RPM based on distance and constant ratio
			switch (targetType)
			{
			case Target::kTopTarget:
				power = 0.2663 * pow(distance, 0.4352); // find equation
				break;
			case Target::kLeftTarget: case Target::kRightTarget:
				power = 0.2663 * pow(distance, 0.4352); // calibrate
				break;
			case Target::kBottomTarget:
				power = 0.2663 * pow(distance, 0.4352); // find equation
				break;
			}
			
			// set limits to motor speed
			if (power > 0.8)
				power = 0.8;
			else if (power < -0.8)
				power = -0.8;
		}
		return power;
	}
	
	// update shooting process
	void updateShooter(float power)
	{
		if (launching)
		{	// Power shooter wheels to speed
			power = SoftStart(shooterMotor1->Get(), power);
			shooterMotor1->Set(power);
			shooterMotor2->Set(power);
		}
		else
		{	// Stop shooter wheels
			power = SoftStart(shooterMotor1->Get(), 0.0);
			shooterMotor1->Set(power);
			shooterMotor2->Set(power);
			launching = false;
		}
	}
	
	Target::TargetType updateTarget(Gamepad::DPadDirection dir = Gamepad::kCenter)
	{
		switch (dir)
		{
		case Gamepad::kUp:
			return Target::kTopTarget;
		case Gamepad::kLeft:
			return Target::kLeftTarget;
		case Gamepad::kRight:
			return Target::kRightTarget;
		case Gamepad::kDown:
			return Target::kBottomTarget;
		default:
			return targetType;
		}
	}
	
	/**
	 * Drive motors with tank steering based on Kinect.
	 */
	void Autonomous(void)
	{
		AxisCamera &camera = AxisCamera::GetInstance("10.8.40.11");
		//kinect = Kinect::GetInstance();
		
		/*A loop is necessary to retrieve the latest Kinect data and update the motors */
		while (IsAutonomous())
		{
			leftSpeed = SoftStart(leftSpeed, leftArm->GetY()*.7);
			rightSpeed = SoftStart(rightSpeed, rightArm->GetY()*.7);
			//drive->TankDrive(leftSpeed, rightSpeed);
			
			//launching = GetGesture(1, kinect);
			updateConveyor(switchTimer1, switchTimer2);
			updateShooter(shooterSpeed);
			updateLever();// GetGesture(3, kinect), GetGesture(3, kinect));
			
			if (camera.IsFreshImage())
			{
				HSLImage *image = camera.GetImage();
				
				// check if image exists
				if (image->GetWidth() && image->GetHeight())
				{
					// Find topmost FRC target
					Target target = Target::FindRectangularTarget(image, targetType);
					delete image;
				
					if (target.m_width > 0 && target.m_height > 0)
					{
						newPosition = target.m_xPos;
						targetWidth = target.m_width;
						targetDistance = getDistance(targetWidth);
						
						// aim and ready shooter
						rotateTurret(newPosition);
						shooterSpeed = getSpeed(targetDistance);
					}
				}
			}
			
			Wait(.01); // Delay 10ms to reduce processing load
		}
	}
		
	/**
	 * Runs the motors with tank steering. 
	 */
	void OperatorControl(void)
	{
		AxisCamera &camera = AxisCamera::GetInstance("10.8.40.11");
		DriverStationLCD *ds = DriverStationLCD::GetInstance();
		Timer *cameraTimer = new Timer();
		
		while (IsOperatorControl()) // loop forever
		{			
			balancing = drivePad->GetButton(2);
			//elevationAngle = gyro->GetAngle();
			if (balancing) // autonomous bridge balancing
			{
				drive->TankDrive(0.1, 0.1);
				//balanceControl->Enable();
				//balanceControl->SetSetpoint(0);
			}
			else // drive with tank drive
			{
				leftSpeed = SoftStart(leftSpeed, drivePad->GetLeftY());
				rightSpeed = SoftStart(rightSpeed, drivePad->GetRightY());
				drive->TankDrive(leftSpeed, rightSpeed);
			}
			
			// update conveyor, shooter, and lever based on input
			launching = turretPad->GetRightTrigger();
			updateConveyor(switchTimer1, switchTimer2);
			updateShooter(shooterSpeed);
			updateLever(drivePad->GetLeftTrigger(), drivePad->GetRightTrigger());
			
			// update target for shooter and check if auto-aiming
			targetType = updateTarget(turretPad->GetDPad());
			autoAiming = turretPad->GetButton(2);
			
			cameraTimer->Reset();
			cameraTimer->Start();
			
			if (camera.IsFreshImage())
			{
				HSLImage *image = camera.GetImage();
				
				// check if image exists
				if (image->GetWidth() && image->GetHeight())
				{
					// Find topmost FRC target
					Target target = Target::FindRectangularTarget(image, targetType);
					delete image;
					
					if (target.m_width > 0 && target.m_height > 0)
					{
						newPosition = target.m_xPos;
						targetWidth = target.m_width;
						targetDistance = getDistance(targetWidth);
						
						if (autoAiming) // aim and update shooter speed autonomously
						{
							rotateTurret(newPosition);
							shooterSpeed = getSpeed(targetDistance);
						}
					}
				}
			}
			cameraTimer->Stop();
			if (!autoAiming) // aim and update shooter speed manually
			{
				rotateTurret(turretPad->GetLeftX());
				shooterSpeed = getSpeed(turretPad->GetRightY(), true);
			}
			
			// print LCD messages
			ds->PrintfLine(DriverStationLCD::kUser_Line1, "x pos: %.2f", newPosition);
			ds->PrintfLine(DriverStationLCD::kUser_Line2, "target distance: %.2f", targetDistance);
			//ds->PrintfLine(DriverStationLCD::kUser_Line3, "shooter speed: %.2f", shooterSpeed);
			//ds->PrintfLine(DriverStationLCD::kUser_Line3, "gyro angle: %.2f", elevationAngle);
			ds->PrintfLine(DriverStationLCD::kUser_Line3, "lever angle: %.2f", leverAngle);
			ds->PrintfLine(DriverStationLCD::kUser_Line4, "turret angle: %.2f", turretAngle);
			ds->PrintfLine(DriverStationLCD::kUser_Line5, "balls: %d", ballCounter);
			ds->PrintfLine(DriverStationLCD::kUser_Line6, "target: %s", Target::TargetToString(targetType));
			ds->UpdateLCD();
			
			Wait(0.001);	// wait for a motor update time
		}
	}
};

START_ROBOT_CLASS(RobotDemo);

