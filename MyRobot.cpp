#include "WPILib.h"
#include "Gamepad.h"
#include "Target.h"
#include "KinectGestures.h"
#include <AnalogChannel.h>
#include <math.h>

#define M_PI 3.14159 // value of pi
#define THETA 20.5 // value of camera angle (half of FOV)

#define AUTO_WAIT_TIME 5.0 // configure before each match for autonomous
#define AUTO_BALL_COUNT 2 // number of balls in possession at start of hybrid
#define MAX_SPEED 0.9 // maximum shooter speed
#define TURRET_MIN 400.0 // minimum turret angle from pot
#define TURRET_MAX 610.0 // maximum turret angle from pot
#define LEVER_MIN 205.0 // minimum lever angle from pot
#define LEVER_MAX 300.0 // maximum lever angle from pot

#define FRONT_LEFT 1 // drive motors, don't change
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
	Kinect *kinect; // Kinect
	KinectStick *leftArm, *rightArm; // left and right arm for Kinect
	
	Victor *conveyorMotor; // ball-collecting conveyor
	Jaguar *shooterMotors; // shooter wheels
	Victor *turretMotor; // Window motor that powers Lazy Susan
	Victor *leverMotor; // lever motor
	
	DigitalInput *entranceSwitch; // rotary switch that detects balls in conveyor
	DigitalInput *exitSwitch; // limit switch that detects balls exiting shooter
	AnalogChannel *turretPot; // potentiometer for measuring turretAngle
	AnalogChannel *leverPot; // potentiometer for measuring leverAngle
	
	Timer *switchTimer1; // timer that measures time between counting incoming balls
	Timer *switchTimer2; // timer that measures time between counting exiting balls
	Timer *autoTimer; // timer for autonomous wait time
	
	float leftSpeed, rightSpeed; // speed of left and right drive motors
	float targetDistance; // distance from target in ft
	float newPosition; // target position from center (-1.0 to 1.0)
	float targetWidth; // width in pixels of target
	float turretAngle; // angle of turret and Lazy Susan
	float shooterSpeed; // speed of shooterMotor
	float leverAngle; // angle of lever
	float elevationAngle; // angle of robot from ground
	int ballCounter; // number of balls in possession
	bool switch1Pressed; // if limit switch (ballCounter) is pressed
	bool switch2Pressed; // if exit limit switch is pressed
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
		
		// Initialize motors (Ports 6 & 8 = DEAD?)
		conveyorMotor = new Victor(5); // reversed
		shooterMotors = new Jaguar(7);
		turretMotor = new Victor(8);
		leverMotor = new Victor(10);
		
		// Initialize sensors and timers
		entranceSwitch = new DigitalInput(1);
		exitSwitch = new DigitalInput(2);
		turretPot = new AnalogChannel(2);
		leverPot = new AnalogChannel(3);
		
		switchTimer1 = new Timer();
		switchTimer2 = new Timer();
		autoTimer = new Timer();
		
		// Initialize variables
		shooterSpeed = MAX_SPEED;
		ballCounter = 0;
		autoAiming = false;
		launching = false;
		balancing = false;
		
		targetType = Target::kTopTarget;
		
		// Initialize camera
		AxisCamera &camera = AxisCamera::GetInstance("10.8.40.11");
		camera.WriteResolution(AxisCamera::kResolution_320x240);
		camera.WriteCompression(30);
		camera.WriteBrightness(40);
		Wait(3.0);

		GetWatchdog().SetEnabled(false);
		drive->SetExpiration(10.0);
	}
	
	// limits maximum change in acceleration to protect chains
	float SoftStart(float current, float target, float limit = 0.1, float max = MAX_SPEED)
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
		if (result > max)
			result = max;
		else if (result < -max)
			result = -max;
		return result;
	}
	
	// updates conveyor motor based on number of balls in possession
	void updateConveyor(Timer *timer1, Timer *timer2)
	{
		//const int BALL_LIMIT = 3; // max balls in possession
		
		// update ball counter (based on rotary and limit switches)
		if (!switch1Pressed && entranceSwitch->Get())
		{
			ballCounter++;
			switch1Pressed = true;
			timer1->Start();
		}
		else if (switch1Pressed && !entranceSwitch->Get() && timer1->Get() > 3.0)
		{
			switch1Pressed = false;
			timer1->Reset();
		}
		if (!switch2Pressed && exitSwitch->Get())
		{
			ballCounter--;
			switch2Pressed = true;
			timer2->Start();
		}
		else if (switch2Pressed && !exitSwitch->Get() && timer2->Get() > 3.0)
		{
			switch2Pressed = false;
			timer2->Reset();
		}
		
		// Turn on conveyor if less than 3 balls in possession, unless shooting
		if (turretPad->GetButton(5))// || ballCounter == 0)
			conveyorMotor->Set(-1.0);
		else if (turretPad->GetLeftTrigger())
			conveyorMotor->Set(1.0);
		else
			conveyorMotor->Set(0.0);
	}
	
	void updateLever(bool rotateDown = false, bool rotateUp = false, bool override = false)
	{
		
		leverAngle = leverPot->GetValue();
		// rotate lever if Button 2 pressed
		if (rotateUp && (leverAngle >= LEVER_MIN || override))
			leverMotor->Set(1.0);
		else if (rotateDown && (leverAngle <= LEVER_MAX || override))
			leverMotor->Set(-1.0);
		else
			leverMotor->Set(0.0);
	}
	
	// get distance to target based on camera image's target width in pixels
	float getDistance(float targetWidth)
	{
		float distance = 320.0 / (targetWidth * tan(THETA * M_PI / 180)); // calibrate later
		return distance;
	}
	
	// rotates turret and Lazy Susan to face target based on camera image
	void rotateTurret(float position)
	{		
		turretAngle = turretPot->GetValue();
		if ((turretAngle <= TURRET_MIN && position < 0.0) || (turretAngle >= TURRET_MAX && position > 0.0))
			position = 0.0;
		
		if (fabs(position) <= 0.005)
			position = 0.0;
		
		turretMotor->Set(MAX_SPEED * position / 2);
	}
	
	// returns speed of shooter motor based on distance from target
	float getSpeed(float distance, bool override = false)
	{
		float power;
		
		// if joystick override
		if (override)
		{
			if (distance < -0.1)
				power = shooterSpeed + 0.0001;
			else if (distance > 0.1)
				power = shooterSpeed - 0.0001;
			else
				power = shooterSpeed;
		}
		else
		{
			// adjust shooter RPM based on distance and constant ratio
			switch (targetType)
			{
			case Target::kTopTarget:
				power = 0.23 + 0.23296 * log(distance); // find equation
				break;
			case Target::kLeftTarget: case Target::kRightTarget:
				power = 0.23 + 0.23296 * log(distance);
				break;
			case Target::kBottomTarget:
				power = 0.2663 * pow(distance, 0.4352); // find equation
				break;
			}
		}
		// set limits to motor speed
		if (power > MAX_SPEED)
			power = MAX_SPEED;
		else if (power < -MAX_SPEED)
			power = -MAX_SPEED;
		
		return power;
	}
	
	// update shooting process
	void updateShooter(float power)
	{
		if (launching)
		{	// Power shooter wheels to speed
			power = SoftStart(shooterMotors->Get(), power);
			shooterMotors->Set(power);
		}
		else
		{	// Stop shooter wheels
			power = SoftStart(shooterMotors->Get(), 0.0);
			shooterMotors->Set(power);
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
		kinect = Kinect::GetInstance();
		
		ballCounter = AUTO_BALL_COUNT;
		
		/*A loop is necessary to retrieve the latest Kinect data and update the motors */
		while (IsAutonomous())
		{
			if (kinect->GetNumberOfPlayers() > 0) // if Kinect detects player
			{
				leftSpeed = SoftStart(leftSpeed, leftArm->GetY()*.7);
				rightSpeed = SoftStart(rightSpeed, rightArm->GetY()*.7);
				drive->TankDrive(leftSpeed, rightSpeed);
				
				launching = GetGesture(1, kinect);
				updateLever(GetGesture(3, kinect), GetGesture(4, kinect));
			}
			else // run autonomous code
			{
				if (autoTimer->Get() == 0.0)
					autoTimer->Start();
				else if (autoTimer->Get() >= AUTO_WAIT_TIME)
				{
					conveyorMotor->Set(-1.0);
					launching = true;
				}
			}
			
			//updateConveyor(switchTimer1, switchTimer2);
			updateShooter(shooterSpeed);
			
			if (camera.IsFreshImage() && !launching)
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
		
		conveyorMotor->Set(0.0);
		autoTimer->Stop();
	}
		
	/**
	 * Runs the motors with tank steering. 
	 */
	void OperatorControl(void)
	{
		AxisCamera &camera = AxisCamera::GetInstance("10.8.40.11");
		DriverStationLCD *ds = DriverStationLCD::GetInstance();
		Timer *cameraTimer = new Timer();
		
		launching = false;
		
		while (IsOperatorControl()) // loop forever
		{			
			balancing = drivePad->GetButton(2);
			if (balancing) // autonomous bridge balancing
			{
				leftSpeed = SoftStart(leftSpeed, -0.2);
				rightSpeed = SoftStart(rightSpeed, -0.2);
			}
			else // drive with tank drive
			{
				leftSpeed = SoftStart(leftSpeed, drivePad->GetLeftY());
				rightSpeed = SoftStart(rightSpeed, drivePad->GetRightY());
			}
			drive->TankDrive(leftSpeed, rightSpeed);
			
			// update conveyor, shooter, and lever based on input
			launching = turretPad->GetButton(6);
			updateConveyor(switchTimer1, switchTimer2);
			updateLever(drivePad->GetLeftTrigger(), drivePad->GetRightTrigger(),
					drivePad->GetButton(3));
			
			// update target for shooter and check if auto-aiming
			targetType = updateTarget(turretPad->GetDPad());
			if (turretPad->GetButton(1))
				shooterSpeed = 0.64;
			else if (turretPad->GetButton(2))
				shooterSpeed = -0.90;
			else if (turretPad->GetButton(3))
				shooterSpeed = 0.78;
			else if (turretPad->GetButton(4))
				shooterSpeed = MAX_SPEED;
			autoAiming = turretPad->GetRightTrigger();
			
			cameraTimer->Reset();
			cameraTimer->Start();
			
			if (autoAiming && camera.IsFreshImage())
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
						
						// aim and update shooter speed autonomously
						rotateTurret(newPosition);
						shooterSpeed = getSpeed(targetDistance);
					}
				}
			}
			cameraTimer->Stop();
			if (!autoAiming) // aim and update shooter speed manually
			{
				rotateTurret(turretPad->GetLeftX());
				shooterSpeed = getSpeed(turretPad->GetRightY(), true);
			}
			updateShooter(shooterSpeed);
			
			// print LCD messages
			ds->PrintfLine(DriverStationLCD::kUser_Line1, "x pos: %.2f", newPosition);
			ds->PrintfLine(DriverStationLCD::kUser_Line2, "target distance: %.2f", targetDistance);
			ds->PrintfLine(DriverStationLCD::kUser_Line3, "shooter speed: %.2f", shooterSpeed);
			//ds->PrintfLine(DriverStationLCD::kUser_Line6, "lever angle: %.2f", leverAngle);
			ds->PrintfLine(DriverStationLCD::kUser_Line4, "turret angle: %.2f", turretAngle);
			ds->PrintfLine(DriverStationLCD::kUser_Line5, "balls: %d", ballCounter);
			//ds->PrintfLine(DriverStationLCD::kUser_Line6, "vision time: %.5f", cameraTimer->Get());
			ds->PrintfLine(DriverStationLCD::kUser_Line6, "target: %s", Target::TargetToString(targetType));
			ds->UpdateLCD();
			
			Wait(0.001);	// wait for a motor update time
		}
	}
};

START_ROBOT_CLASS(RobotDemo);

