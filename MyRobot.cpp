#include "WPILib.h"
#include "Gamepad.h"
#include "Target.h"
#include <AnalogChannel.h>
#include <math.h>

#define M_PI 3.14159 // value of pi
#define THETA 20.5 // value of camera angle (half of FOV)

#define AUTO_WAIT_TIME 0.0 // configure before each match for autonomous (+3 sec to shoot)
#define AUTO_SHOOT_TIME 10.0 // time in seconds for shooting
#define AUTO_BACK_TIME 3.0 // time in seconds backing to bridge
#define AUTO_LEVER_TIME 0.0 // time in seconds for using lever
#define AUTO_BRIDGE 1 // bridge to back toward (1 = coopertition, 2 = alliance)
#define AUTO_FEED false // reverse conveyor in autonomous instead of shooting

#define AUTO_DISTANCE 4.2 // wheel revolutions: from key to bridge
#define AUTO_BALL_COUNT 2 // number of balls in possession at start of hybrid

#define MAX_SPEED 0.9 // maximum shooter speed
#define FENDER_SPEED 0.565 // speed to shoot from fender for middle hoops
#define KEY_SPEED 0.83 // speed to shoot from key for top hoop
#define AUTO_SPEED 0.83 // starting speed for autonomous: 0.8 = front, 0.85 = midkey

#define TURRET_MIN 120.0 // minimum turret angle from pot
#define TURRET_MAX 355.0 // maximum turret angle from pot
#define TURRET_CENTER 245.0 // center value for turret angle from pot
#define LEVER_MIN 165.0 // minimum lever angle from pot
#define LEVER_MAX 265.0 // maximum lever angle from pot

#define FRONT_LEFT 1 // drive motors, don't change
#define REAR_LEFT 2
#define FRONT_RIGHT 3
#define REAR_RIGHT 4

#define TURRET_P 0.3 // proportional value for turret PD; adjust later
#define TURRET_D 0.05 // derivative value for turret PD; adjust later

/**
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */ 
class MyRobot : public SimpleRobot
{
	RobotDrive *drive; // robot drive system
	Encoder *leftEncoder, *rightEncoder; // drive encoders
	Gamepad *drivePad, *turretPad; // gamepads
	Kinect *kinect; // Kinect
	KinectStick *leftArm, *rightArm; // left and right arm for Kinect
	
	Victor *conveyorMotor; // ball-collecting conveyor
	Jaguar *shooterMotors; // shooter wheels
	Victor *turretMotor; // Window motor that powers Lazy Susan
	Victor *leverMotor; // lever motor
	
	//DigitalInput *entranceSwitch; // rotary switch that detects balls in conveyor
	DigitalInput *exitSwitch; // limit switch that detects balls exiting shooter
	AnalogChannel *turretPot; // potentiometer for measuring turretAngle
	AnalogChannel *leverPot; // potentiometer for measuring leverAngle
	
	Timer *switchTimer1; // timer that measures time between counting incoming balls
	Timer *switchTimer2; // timer that measures time between counting exiting balls
	Timer *leverTimer; // timer that measures time after lever deployment
	
	float leftSpeed, rightSpeed; // speed of left and right drive motors
	float targetDistance; // distance from target in ft
	float newPosition; // target position from center (-1.0 to 1.0)
	float targetWidth; // width in pixels of target
	float turretAngle; // angle of turret and Lazy Susan
	float shooterSpeed; // speed of shooterMotor
	float leverAngle; // angle of lever
	float elevationAngle; // angle of robot from ground
	float lastError; // previous error in auto-targeting
	int ballCounter; // number of balls in possession
	//bool switch1Pressed; // if limit switch (ballCounter) is pressed
	bool switch2Pressed; // if exit limit switch is pressed
	bool autoAiming; // if camera autonomously aims turret
	bool launching; // if firing ball
	
	typedef enum {
		kShooting, kBack, kLever, kStopped, kTeleOp
	} RobotState; // robot state
	RobotState robotState;
	
	Target::TargetType targetType; // which target to aim for

public:
	MyRobot(void)
	{
		// Initialize drivetrain motors
		drive = new RobotDrive(FRONT_LEFT, REAR_LEFT, FRONT_RIGHT, REAR_RIGHT);
		leftEncoder = new Encoder(3, 4); // configure later- Digital Input
		rightEncoder = new Encoder(5, 6);
		
		const double FEET_PP = (0.5 * M_PI) / 250 / 4.67; // feet per pulse
		leftEncoder->SetDistancePerPulse(FEET_PP); // 250 pulses per revolution
		rightEncoder->SetDistancePerPulse(FEET_PP); // gearing = 4.67:1
		
		// Initialize gamepads and Kinect
		drivePad = new Gamepad(1);
		turretPad = new Gamepad(2);
		leftArm = new KinectStick(1);
		rightArm = new KinectStick(2);
		
		// Initialize motors (Sidecar port 6 = dead?)
		conveyorMotor = new Victor(5); // reversed
		shooterMotors = new Jaguar(7);
		turretMotor = new Victor(8);
		leverMotor = new Victor(10);
		
		// Initialize sensors and timers (Analog slot 3 = dead?)
		//entranceSwitch = new DigitalInput(1);
		exitSwitch = new DigitalInput(2);
		turretPot = new AnalogChannel(2);
		leverPot = new AnalogChannel(4);
		
		switchTimer1 = new Timer();
		switchTimer2 = new Timer();
		leverTimer = new Timer();
		
		// Initialize variables
		shooterSpeed = AUTO_SPEED;
		lastError = 0.0;
		ballCounter = 0;
		autoAiming = false;
		launching = false;
		
		robotState = kStopped;
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
	
	// returns a string displaying robot state
	char* StateToString(RobotState state)
	{
		char* string = "";
		switch (state)
		{
		case kStopped:
			string = "stopped";
			break;
		case kShooting:
			string = "shooting";
			break;
		case kBack:
			string = "backing";
			break;
		case kLever:
			string = "lever";
			break;
		case kTeleOp:
			string = "teleop";
			break;
		}
		return string;
	}
	
	// limits maximum change in acceleration to protect chains
	float SoftStart(float current, float target, float limit = 0.6, float max = MAX_SPEED)
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
		/*if (!switch1Pressed && entranceSwitch->Get())
		{
			ballCounter++;
			switch1Pressed = true;
			timer1->Start();
		}
		else if (switch1Pressed && !entranceSwitch->Get() && timer1->Get() > 2.0)
		{
			switch1Pressed = false;
			timer1->Reset();
		}*/
		if (!switch2Pressed && exitSwitch->Get())
		{
			ballCounter--;
			switch2Pressed = true;
			timer2->Start();
		}
		else if (switch2Pressed && !exitSwitch->Get() && timer2->Get() > 2.0)
		{
			switch2Pressed = false;
			timer2->Reset();
		}
		
		// Turn on conveyor if less than 3 balls in possession, unless shooting
		if (turretPad->GetButton(5))// || ballCounter == 0)
			conveyorMotor->Set(1.0);
		else if (turretPad->GetLeftTrigger())
			conveyorMotor->Set(-1.0);
		else
			conveyorMotor->Set(0.0);
	}
	
	void updateLever(bool rotateDown = false, bool rotateUp = false, bool override = true)
	{
		leverAngle = leverPot->GetValue();
		// rotate lever if left trigger pressed
		if (rotateUp && (leverAngle >= LEVER_MIN || override))
		{
			leverMotor->Set(1.0);
			leverTimer->Reset();
		}
		else if (rotateDown && (leverAngle <= LEVER_MAX || override))
		{
			leverMotor->Set(-1.0);
			leverTimer->Reset();
		}
		else
		{
			if (leverTimer->Get() > 10.0 && leverAngle > (LEVER_MIN + 10.0)) // if untouched for 10 seconds
				leverMotor->Set(1.0);
			else
			{
				if (leverTimer->Get() == 0.0)
					leverTimer->Start();
				leverMotor->Set(0.0);
			}
		}
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
		//if ((turretAngle <= TURRET_MIN && position < 0.0) || (turretAngle >= TURRET_MAX && position > 0.0))
			//position = 0.0;
		
		float derivative = position - lastError;
		float output = (TURRET_P * position) + (TURRET_D * derivative);
		lastError = position;
		
		turretMotor->Set(output);
	}
	
	// returns speed of shooter motor based on distance from target
	float getSpeed(float distance, bool override = false)
	{
		float power;
		
		// if joystick override
		if (override)
		{
			if (distance < -0.25) // right joystick up
				power = shooterSpeed + 0.0001;
			else if (distance > 0.25) // right joystick down
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
		
		// reset encoders and start counting pulses
		leftEncoder->Reset();
		rightEncoder->Reset();
		leftEncoder->Start();
		rightEncoder->Start();
		
		double encoderDistance = 0.0;
		
		Timer *autoTimer = new Timer();
		autoTimer->Start();
		ballCounter = AUTO_BALL_COUNT;
		shooterSpeed = AUTO_SPEED;
		
		bool kinectOverride = false;
		
		/*A loop is necessary to retrieve the latest Kinect data and update the motors */
		while (IsAutonomous())
		{
			encoderDistance = max(leftEncoder->GetDistance(), rightEncoder->GetDistance());
			
			if (kinect->GetNumberOfPlayers() > 0) // if Kinect detects player
			{
				if (autoTimer->Get() < AUTO_WAIT_TIME)
					robotState = kStopped;
				else if (autoTimer->Get() < AUTO_WAIT_TIME + AUTO_SHOOT_TIME && !kinectOverride)
					robotState = kShooting;
				
				if (fabs(leftArm->GetY()) < 0.1 && fabs(rightArm->GetY()) < 0.1) // T mode
					robotState = kStopped;
				//else if (leftArm->GetRawButton(1) || leftArm->GetRawButton(2)) // left leg out, R mode
					//robotState = kLever;
				else if (leftArm->GetY() < -0.9 && rightArm->GetY() < -0.9) // A mode
				{
					robotState = kBack;
					kinectOverride = true;
				}
			}
			else // run autonomous code
			{
				if (autoTimer->Get() < AUTO_WAIT_TIME)
					robotState = kStopped;
				else if (autoTimer->Get() < AUTO_WAIT_TIME + AUTO_SHOOT_TIME)
					robotState = kShooting;
				else if (autoTimer->Get() < AUTO_WAIT_TIME + AUTO_SHOOT_TIME + AUTO_BACK_TIME)
					robotState = kBack;
				else
					robotState = kStopped;
			}
			
			switch (robotState)
			{				
			case kBack: // if robot is backing
				conveyorMotor->Set(0.0);
				if (fabs(encoderDistance) < AUTO_DISTANCE)
				{
					updateLever(false, false); // stop lever
					if (AUTO_BRIDGE == 1) // coopertition bridge (middle position)
						drive->TankDrive(0.65, 0.65);
					else if (AUTO_BRIDGE == 2) // alliance bridge (right position)
						drive->TankDrive(0.75, 0.75);
					else
						drive->TankDrive(0.0, 0.0);
				}
				else if (encoderDistance >= AUTO_DISTANCE && kinectOverride)
				{
					drive->TankDrive(0.0, 0.0);
					updateLever(true, false); // lower lever
				}
				break;
			
			case kLever: // if robot is using lever
				conveyorMotor->Set(0.0);
				drive->TankDrive(0.0, 0.0);
				updateLever(true, false); // lower lever
				break;
				
			case kShooting: // if robot is shooting
				drive->TankDrive(0.0, 0.0);
				if (AUTO_FEED == true)
					conveyorMotor->Set(-1.0);
				else
					conveyorMotor->Set(1.0);
				if (autoTimer->Get() > 5.0 + AUTO_WAIT_TIME)
					shooterSpeed = AUTO_SPEED + 0.007;
				else
					shooterSpeed = AUTO_SPEED;
				break;
				
			case kStopped: default: // if robot is stopped
				drive->TankDrive(0.0, 0.0);
				conveyorMotor->Set(0.0);
				updateLever(false, false); // stop lever
				//updateLever(false, true); // retract lever if necessary
				break;
			}
			
			launching = (robotState == kShooting);
			updateShooter(shooterSpeed);
			
			if (camera.IsFreshImage() && robotState == kShooting && !launching && newPosition != 0.0)
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
			DriverStationLCD *ds = DriverStationLCD::GetInstance();
			ds->PrintfLine(DriverStationLCD::kUser_Line1, "state: %s", StateToString(robotState));
			ds->PrintfLine(DriverStationLCD::kUser_Line2, "auto time: %f", autoTimer->Get());
			ds->PrintfLine(DriverStationLCD::kUser_Line3, "encoder: %f", encoderDistance);
			ds->PrintfLine(DriverStationLCD::kUser_Line4, "shooter speed: %.2f", shooterSpeed);
			ds->PrintfLine(DriverStationLCD::kUser_Line5, "balls: %d", ballCounter);
			ds->UpdateLCD();
			
			Wait(.01); // Delay 10ms to reduce processing load
		}
		
		autoTimer->Reset();
		robotState = kStopped;
	}
		
	/**
	 * Runs the motors with tank steering. 
	 */
	void OperatorControl(void)
	{
		AxisCamera &camera = AxisCamera::GetInstance("10.8.40.11");
		DriverStationLCD *ds = DriverStationLCD::GetInstance();
		Timer *cameraTimer = new Timer();
		
		robotState = kTeleOp;
		launching = false;
		
		while (IsOperatorControl()) // loop forever
		{
			if (drivePad->GetButton(5)) // bridge balancing: crawl backward
			{
				leftSpeed = SoftStart(leftSpeed, -0.5);
				rightSpeed = SoftStart(rightSpeed, -0.5);
			}
			else if (drivePad->GetButton(6)) // bridge balancing: crawl forward
			{
				leftSpeed = SoftStart(leftSpeed, 0.5);
				rightSpeed = SoftStart(rightSpeed, 0.5);
			}
			else // drive with tank drive
			{
				leftSpeed = SoftStart(leftSpeed, drivePad->GetLeftY(), 0.6, 1.0);
				rightSpeed = SoftStart(rightSpeed, drivePad->GetRightY(), 0.6, 1.0);
			}
			drive->TankDrive(leftSpeed, rightSpeed);
			
			// update conveyor, shooter, and lever based on input
			launching = turretPad->GetButton(6) || turretPad->GetRightTrigger();
			updateConveyor(switchTimer1, switchTimer2);
			updateLever(drivePad->GetLeftTrigger(), drivePad->GetRightTrigger());
			
			// update target for shooter and check if auto-aiming
			targetType = updateTarget(turretPad->GetDPad());
			if (turretPad->GetButton(1))
				shooterSpeed = FENDER_SPEED;
			else if (turretPad->GetButton(3))
				shooterSpeed = KEY_SPEED;
			else if (turretPad->GetButton(4))
				shooterSpeed = MAX_SPEED;
			autoAiming = turretPad->GetButton(2);
			
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
				if (turretPad->GetRightTrigger())
					shooterSpeed = -MAX_SPEED / 2;
				else
					shooterSpeed = getSpeed(turretPad->GetRightY(), true);
			}
			updateShooter(shooterSpeed);
			
			// print LCD messages
			ds->PrintfLine(DriverStationLCD::kUser_Line1, "x pos: %.2f", newPosition);
			ds->PrintfLine(DriverStationLCD::kUser_Line2, "target distance: %.2f", targetDistance);
			ds->PrintfLine(DriverStationLCD::kUser_Line3, "shooter speed: %.2f", shooterSpeed);
			ds->PrintfLine(DriverStationLCD::kUser_Line4, "target width: %.2f", targetWidth);
			ds->PrintfLine(DriverStationLCD::kUser_Line5, "lever angle: %.2f", leverAngle);
			ds->PrintfLine(DriverStationLCD::kUser_Line6, "turret angle: %.2f", turretAngle);
			//ds->PrintfLine(DriverStationLCD::kUser_Line5, "balls: %d", ballCounter);
			//ds->PrintfLine(DriverStationLCD::kUser_Line6, "vision time: %.5f", cameraTimer->Get());
			//ds->PrintfLine(DriverStationLCD::kUser_Line6, "target: %s", Target::TargetToString(targetType));
			ds->UpdateLCD();
			
			Wait(0.001);	// wait for a motor update time
		}
	}
};

START_ROBOT_CLASS(MyRobot);
