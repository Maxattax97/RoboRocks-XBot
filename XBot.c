#pragma config(Sensor, dgtl1,  PRT_gunLeftQuad, sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  PRT_gunRightQuad, sensorQuadEncoder)
#pragma config(Sensor, dgtl5,  PRT_sonar,      sensorSONAR_raw)
#pragma config(Sensor, dgtl9,  PRT_ledGun,     sensorLEDtoVCC)
#pragma config(Sensor, dgtl10, PRT_ledR,       sensorLEDtoVCC)
#pragma config(Sensor, dgtl11, PRT_ledY,       sensorLEDtoVCC)
#pragma config(Sensor, dgtl12, PRT_ledG,       sensorLEDtoVCC)
#pragma config(Motor,  port1,           PRT_feedLower, tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           PRT_feedUpper, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           PRT_wheelFrontRight, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           PRT_wheelBackLeft, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           PRT_wheelBackRight, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           PRT_gunLeft1,  tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           PRT_gunLeft2,  tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           PRT_gunRight1, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           PRT_gunRight2, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port10,          PRT_wheelFrontLeft, tmotorVex393_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//////////////////////////////////////////////////////////////////////////////////////
//                                  --= X BOT =--                                   //
//////////////////////////////////////////////////////////////////////////////////////
// The purpose of this file is to link all of the modules together into a working,  //
// intelligent robot. The robot starts by calibrating its systems, then enables its //
// autonomous mode. After this, user input is converted to instructions during the  //
// user control period.                                                             //
//////////////////////////////////////////////////////////////////////////////////////

#pragma platform(VEX)

//Competition Control and Duration Settings
#pragma competitionControl(Competition)
#pragma autonomousDuration(15)
#pragma userControlDuration(120)

//Main competition background code...do not modify!
#include "Vex_Competition_Includes.c"

#define DEBUG 0

#define ROBOT "Robot"
#define EMULATOR "Emulator"
#define PROGRAMMING_CHALLENGE 0

#if (_TARGET == EMULATOR)
#warning "###!!!### RUNNING IN EMULATOR MODE ###!!!###"
#define EMULATING 1
#else
#define EMULATING 0
#endif

#if (DEBUG == 1)
#warning "###!!!### RUNNING IN DEBUG MODE ###!!!###"
#endif

#if (PROGRAMMING_CHALLENGE == 1)
#warning "###!!!### SET TO RUN IN PROGRAMMING CHALLENGE MODE ###!!!###"
#endif

//////////////////
// DEPENDENCIES //
//////////////////

#include "Utilities.c"
#include "FiringTable.c"
#include "BlinkModule.c"
#include "BatteryModule.c"
#include "SonarModule.c"
#include "TrajectoryModule.c"
#include "DriverControlModule.c"
#include "GunModule.c"
#include "PIDLoopModule.c"
#include "AutonomousModule.c"

///////////////
// TODO LIST //
/*/////////////

Monitor battery backup.
Investigate power expander status port.

And as always, DOCUMENT MORE.

*///////////////////
// USER VARIABLES //
////////////////////

const float USR_DEFAULT_SPEED = 1475;
const float USR_DEFAULT_RANGE = 13 * 12;
const float USR_RANGE_LARGE_INCREMENT = 2 * 12; // 1 floor tile.
const float USR_RANGE_SMALL_INCREMENT = 0.5 * 12; // Quarter floor tile.
short USR_pingId = -1;
short USR_sonarId = -1;
short USR_reverseMultiplier = 1; // Setup the wheel reversal.
float USR_targetRange = USR_DEFAULT_RANGE; // In feet, how far the ball should go if fired.
float USR_targetSpeed = USR_DEFAULT_SPEED;

//////////////////////////
// PRE-AUTONOMOUS SETUP //
//////////////////////////

void pre_auton()
{
	writeDebugStreamLine("[Mode]: Setting up the robot...");
	// Set bStopTasksBetweenModes to false if you want to keep user created tasks running between
	// Autonomous and Tele-Op modes. You will need to manage all user created tasks if set to false.
	bStopTasksBetweenModes = false; // This must be disabled to allow the PID Loop.

	// Reset Timer 1.
	clearTimer(T1);

	// Initialize the blink debug handler.
	// Each blink task has an ID that must be held in order to stop the task later.
	startTask(LED_blink);
	short id = LED_startBlinkTask(Info, Medium);

	// Initialize the driver configuration and button "event" handler.
	startTask(DRV_buttonHandler);

	// Initialize the battery monitor.
	startTask(BAT_monitor);

	// Reset sensors.
	SensorType[PRT_gunLeftQuad] = sensorNone; // JPearman said to do this.
	SensorType[PRT_gunLeftQuad] = sensorQuadEncoder; // Resets encoders in RobotC, fixes error.
	SensorType[PRT_gunRightQuad] = sensorNone;
	SensorType[PRT_gunRightQuad] = sensorQuadEncoder;
	SensorValue[PRT_gunLeftQuad] = 0;
	SensorValue[PRT_gunRightQuad] = 0;

	// Start sonar updates.
	startTask(SNR_update);

	// Initialize the gun controller.
	startTask(GUN_controller);

	// Initialize the PID loop.
	startTask(PID_controller);

	// Prepare the Autonomous.
	//AUT_calibrate();

	// Enable the lower feed for 8 seconds to preload balls.
	//AUT_feedLower(127, 15);

	wait1Msec(500);
	LED_stopBlinkTask(id);
	writeDebugStreamLine("[Mode]: Setup complete!");
#if (EMULATING == 1)
	startTask(usercontrol);
#endif
}

/////////////////////
// AUTONOMOUS MODE //
/////////////////////

task autonomous()
{
#if (PROGRAMMING_CHALLENGE == 0)
	short id = LED_startBlinkTask(Info, Slow);
	writeDebugStreamLine("[Mode]: Autonomous mode enabled!");
	writeDebugStreamLine("[Auton]: Warming guns...");
	PID_target[Left] = USR_DEFAULT_SPEED; //TRJ_angularSpeedAtRange(13 * 12);
	PID_target[Right] = PID_target[Left];
	while (!PID_ready) {
		wait1Msec(50);
	}
	writeDebugStreamLine("[Auton]: Guns warmed.");
	//AUT_feedUpper(64);
	//AUT_feedLower(64);
	PID_firingBall = true;
	for (int i = 0; i < 3; i++) {
			wait1Msec(750);
			writeDebugStreamLine("[Auton]: Firing ball %i...", i + 1);
			PID_firingBall = true;
			AUT_feedLower(127);
			AUT_feedUpper(127);
			if (i + 1 == 4) {
				// Wiggle the last ball out.
				AUT_surge(64, 0.5);
				AUT_surge(-64, 0.5);
			}
			while (PID_firingBall == true) {
				wait1Msec(50);
			}
			writeDebugStreamLine("[Auton]: Ball fired!");
			AUT_feedLower(0);
			AUT_feedUpper(0);
	}
	/*PID_firingBall = true;
	AUT_feedUpper(127);
	while (PID_firingBall == true) {
		wait1Msec(50);
	}
	writeDebugStreamLine("[Auton]: Ball fired!");
	AUT_feedUpper(0);
	for (int i = 0; i < 3; i++) {
			while (!PID_ready) { // Max power changed
				wait1Msec(50);
			}
			writeDebugStreamLine("[Auton]: Firing ball %i...", i + 2);
			PID_firingBall = true;
			AUT_feedLower(127);
			AUT_feedUpper(127);
			if (i + 2 == 4) {
				// Wiggle the last ball out.
				AUT_surge(64, 0.5);
				AUT_surge(-64, 0.5);
			}
			while (PID_firingBall == true) {
				wait1Msec(50);
			}
			writeDebugStreamLine("[Auton]: Ball fired!");
			AUT_feedLower(0);
			AUT_feedUpper(0);
	}
	writeDebugStreamLine("[Auton]: Shutting down.");
	AUT_shutDown();
	writeDebugStreamLine("[Mode]: Autonomous mode disabled.");
	LED_stopBlinkTask(id);*/
#else
	short id = LED_startBlinkTask(Info, Slow);
	writeDebugStreamLine("[Mode]: Autonomous (programming skills) mode enabled!");
	writeDebugStreamLine("[Auton]: Warming guns...");
	PID_target[Left] = USR_DEFAULT_SPEED; //TRJ_angularSpeedAtRange(13 * 12);
	PID_target[Right] = PID_target[Left];
	//while (!PID_ready) {
	//	wait1Msec(50);
	//}
	//AUT_feedLower(-64);
	/*ballsFired = 0;
	PID_firingBall = true;
	while (true) {
		if (PID_firingBall == false) {
			ballsFired++;
			PID_firingBall = true;
		}
		if (ballsFired >= 32 ) {
			break;
		}
		wait1Msec(100);
	}*/
	//wait1Msec(45000);
	//writeDebugStreamLine("Moving!");
	//AUT_rotate(-127, 0.5);
	//AUT_surge(127, 5);
	//AUT_rotate(127, 2);
	while (true) {
		// Stall until the end of programming skills.
		if (SNR_distanceInches <= 2) {
			motor[PRT_feedUpper] = 127;
		} else {
			motor[PRT_feedUpper] = 0;
		}
		wait1Msec(50);
	}
	writeDebugStreamLine("[Mode]: Autonomous (programming skills) mode disabled.");
	AUT_shutDown();
	LED_stopBlinkTask(id);
#endif
}

///////////////////////////
//// USER CONTROL MODE ////
///////////////////////////

// For manual, computer-based value editing.
bool USR_OVERRIDE_USER_CONTROL = false;
bool USR_FORCE_AUTON = false;
#if (DEBUG == 1)
USR_OVERRIDE_USER_CONTROL = true;
#endif

task usercontrol()
{
	writeDebugStreamLine("[Mode]: User Control mode enabled!");
	// Reset the PID target.
	PID_target[Left] = 0;
	PID_target[Right] = 0;
	// Continually check for user input.
	while (!USR_OVERRIDE_USER_CONTROL) {
		// Feed Control
		if ((DRV_config[FeedLowerIn] != UNASSIGNED && vexRT[DRV_config[FeedLowerIn]] == true) ||
			(DRV_config[FeedLowerInSecondary] != UNASSIGNED && vexRT[DRV_config[FeedLowerInSecondary]] == true)) {
			motor[PRT_feedLower] = 127;
		} else if ((DRV_config[FeedLowerOut] != UNASSIGNED && vexRT[DRV_config[FeedLowerOut]] == true) ||
			(DRV_config[FeedLowerOutSecondary] != UNASSIGNED && vexRT[DRV_config[FeedLowerOutSecondary]] == true)) {
			motor[PRT_feedLower] = -127;
		} else {
			motor[PRT_feedLower] = 0;
		}
		if ((DRV_config[FeedUpperIn] != UNASSIGNED && vexRT[DRV_config[FeedUpperIn]] == true) ||
			(DRV_config[FeedUpperInSecondary] != UNASSIGNED && vexRT[DRV_config[FeedUpperInSecondary]] == true)) {
			motor[PRT_feedUpper] = 127;
		} else if ((DRV_config[FeedUpperOut] != UNASSIGNED && vexRT[DRV_config[FeedUpperOut]] == true) ||
			(DRV_config[FeedUpperOutSecondary] != UNASSIGNED && vexRT[DRV_config[FeedUpperOutSecondary]] == true)) {
			motor[PRT_feedUpper] = -127;
		} else {
			motor[PRT_feedUpper] = 0;
		}

		// Gun Control
		if (DRV_controllerButtonsDown[GunIncrement] == true) {
			if (PID_enabled == true)
				//USR_targetRange += USR_RANGE_LARGE_INCREMENT;
				USR_targetSpeed += 125;
			else if (GUN_enabled == true)
				GUN_maxMotorPower += GUN_LARGE_INCREMENT;
			DRV_controllerButtonsDown[GunIncrement] = false;
		} else if (DRV_controllerButtonsDown[GunDecrement] == true) {
			if (PID_enabled == true)
				//USR_targetRange -= USR_RANGE_LARGE_INCREMENT;
				USR_targetSpeed -= 125;
			else if (GUN_enabled == true)
				GUN_maxMotorPower -= GUN_LARGE_INCREMENT;
			DRV_controllerButtonsDown[GunDecrement] = false;
		}

		if (DRV_controllerButtonsDown[GunSmallIncrement] == true) {
			if (PID_enabled == true)
				//USR_targetRange += USR_RANGE_SMALL_INCREMENT;
				USR_targetSpeed += 25;
			else if (GUN_enabled == true)
				GUN_maxMotorPower += GUN_SMALL_INCREMENT;
			DRV_controllerButtonsDown[GunSmallIncrement] = false;
		} else if (DRV_controllerButtonsDown[GunSmallDecrement] == true) {
			if (PID_enabled == true)
				//USR_targetRange -= USR_RANGE_SMALL_INCREMENT;
				USR_targetSpeed -= 25;
			else if (GUN_enabled == true)
				GUN_maxMotorPower -= GUN_SMALL_INCREMENT;
			DRV_controllerButtonsDown[GunSmallDecrement] = false;
		}

		if (DRV_controllerButtonsDown[GunResetTarget] == true) {
			if (PID_enabled == true)
				//USR_targetRange = USR_DEFAULT_RANGE;
				USR_targetSpeed = USR_DEFAULT_SPEED;
			else if (GUN_enabled == true)
				GUN_maxMotorPower = GUN_DEFAULT_POWER;
			DRV_controllerButtonsDown[GunResetTarget] = false;
		}

		if (PID_target[Left] != 0 || PID_target[Right] != 0) {
			/*float dist = TRJ_angularSpeedAtRange(USR_targetRange);
			if (dist != SNR_INVALID) {
				PID_target[Left] = dist;
				PID_target[Right] = dist;
			}*/
			PID_target[Left] = USR_targetSpeed;
			PID_target[Right] = USR_targetSpeed;
		}

		if (DRV_controllerButtonsDown[GunWarm] == true) {
			if (PID_enabled == true) {
				if (PID_target[Left] != 0 || PID_target[Right] != 0) {
					PID_target[Left] = 0;
					PID_target[Right] = 0;
					//USR_targetRange = USR_DEFAULT_RANGE;
					USR_targetSpeed = USR_DEFAULT_SPEED;
				} else {
					PID_target[Left] = TRJ_angularSpeedAtRange(USR_targetRange);
					PID_target[Right] = PID_target[Left];
				}
			} else if (GUN_enabled == true) {
				GUN_maxMotorPower = GUN_DEFAULT_POWER;
				GUN_warming = !GUN_warming;
			}
			DRV_controllerButtonsDown[GunWarm] = false;
		}

		/* Sonar Capture
		if (vexRT[DRV_config[SonarCapture]] == true) {
			float dist = SNR_distanceInches;
			if (TRJ_validRange(dist)) {
				USR_targetRange = dist;
				if (PID_target[Left] != 0 || PID_target[Right] != 0) {
					PID_target[Left] = TRJ_angularSpeedAtRange(USR_targetRange);
					PID_target[Right] = PID_target[Left];
				}
			} else if (USR_sonarId == -1) {
				USR_sonarId = LED_startBlinkTask(Warning, Solid);
			}
		} else if (USR_sonarId != -1) {
			if (LED_stopBlinkTask(USR_sonarId))
				USR_sonarId = -1;
		}*/

		// Wheel Control
		if (DRV_controllerButtonsDown[ToggleMirror] == true) {
			USR_reverseMultiplier *= -1; // Flip the multiplier's sign.
			DRV_controllerButtonsDown[ToggleMirror] = false; // Consume event.
		}

		if (DRV_config[OmniLeft] != UNASSIGNED && DRV_config[OmniRight] != UNASSIGNED) {
			// 2 channel (tank) drive for OMNI WHEELS enabled.
			motor[PRT_wheelFrontLeft]  = DRV_trimChannel(OmniLeft)  * USR_reverseMultiplier;
			motor[PRT_wheelBackLeft]   = DRV_trimChannel(OmniLeft)  * USR_reverseMultiplier;
			motor[PRT_wheelFrontRight] = DRV_trimChannel(OmniRight) * USR_reverseMultiplier;
			motor[PRT_wheelBackRight]  = DRV_trimChannel(OmniRight) * USR_reverseMultiplier;
		} else if (DRV_config[OmniForward] != UNASSIGNED && DRV_config[OmniRotate] != UNASSIGNED) {
			// 2 channel (smart) drive for OMNI WHEELS enabled.
			if ((DRV_config[OmniMirrorForward] != UNASSIGNED && DRV_config[OmniMirrorRotate] != UNASSIGNED)
				&& (DRV_trimChannel(OmniForward) != 0 || DRV_trimChannel(OmniRotate) != 0)) {
				// To prevent setting motors to 0
				motor[PRT_wheelFrontLeft]  = (DRV_trimChannel(OmniForward) + DRV_trimChannel(OmniRotate)) * USR_reverseMultiplier;
				motor[PRT_wheelBackLeft]   = (DRV_trimChannel(OmniForward) + DRV_trimChannel(OmniRotate)) * USR_reverseMultiplier;
				motor[PRT_wheelFrontRight] = (DRV_trimChannel(OmniForward) - DRV_trimChannel(OmniRotate)) * USR_reverseMultiplier;
				motor[PRT_wheelBackRight]  = (DRV_trimChannel(OmniForward) - DRV_trimChannel(OmniRotate)) * USR_reverseMultiplier;
			}
		} else if (DRV_config[MecanumRotate] == UNASSIGNED) {
			// 4 channel (tank) drive for MECANUM WHEELS enabled.
			int threshold = 20;
			int RX = DRV_trimChannel(MecanumRightStrafe);
			int RY = DRV_trimChannel(MecanumRightNormal);
			int LX = DRV_trimChannel(MecanumLeftStrafe);
			int LY = DRV_trimChannel(MecanumLeftNormal);

			if (abs(RX) < threshold) {
				// Straight
				motor[PRT_wheelFrontRight] = RY;
				motor[PRT_wheelBackRight] = RY;
			} else if (RX > threshold) {
				// Right
				motor[PRT_wheelFrontRight] = RX;
				motor[PRT_wheelBackRight] = -RX;
			} else if (RX < -threshold) {
				// Left
				motor[PRT_wheelFrontRight] = RX;
				motor[PRT_wheelBackRight] = -RX;
			} else {
				// Stop
				motor[PRT_wheelFrontRight] = 0;
				motor[PRT_wheelBackRight] = 0;
			}

			if (abs(LX) < threshold) {
				// Straight
				motor[PRT_wheelFrontLeft] = LY;
				motor[PRT_wheelBackLeft] = LY;
			} else if (LX > threshold) {
				// Right
				motor[PRT_wheelFrontLeft] = -LX;
				motor[PRT_wheelBackLeft] = LX;
			} else if (LX < -threshold) {
				// Left
				motor[PRT_wheelFrontLeft] = -LX;
				motor[PRT_wheelBackLeft] = LX;
			} else {
				// Stop
				motor[PRT_wheelFrontLeft] = 0;
				motor[PRT_wheelBackLeft] = 0;
			}
		} else if (DRV_config[MecanumRotate] != UNASSIGNED) {
			// 3 channel (smart) drive for MECANUM WHEELS enabled.
			motor[PRT_wheelFrontRight] 	= DRV_trimChannel(MecanumRightNormal)
				- DRV_trimChannel(MecanumRotate)
				+ DRV_trimChannel(MecanumRightStrafe);
   		motor[PRT_wheelBackRight] 	= DRV_trimChannel(MecanumRightNormal)
   			+ abs(DRV_trimChannel(MecanumRotate))
   			- DRV_trimChannel(MecanumRightStrafe);
    	motor[PRT_wheelFrontLeft] 	= DRV_trimChannel(MecanumLeftNormal)
    		+ DRV_trimChannel(MecanumRotate)
    		- DRV_trimChannel(MecanumLeftStrafe);
    	motor[PRT_wheelBackLeft] 		= DRV_trimChannel(MecanumLeftNormal)
    		+ abs(DRV_trimChannel(MecanumRotate))
    		+ DRV_trimChannel(MecanumLeftStrafe);
		}

		if (DRV_config[OmniMirrorForward] != UNASSIGNED && DRV_config[OmniMirrorRotate] != UNASSIGNED
			&& (DRV_trimChannel(OmniForward) == 0 && DRV_trimChannel(OmniRotate) == 0)) {
			// 2 channel (smart) MIRRORED drive for OMNI WHEELS enabled.
			motor[PRT_wheelFrontLeft]  = (DRV_trimChannel(OmniMirrorForward) + DRV_trimChannel(OmniMirrorRotate)) * -1;
			motor[PRT_wheelBackLeft]   = (DRV_trimChannel(OmniMirrorForward) + DRV_trimChannel(OmniMirrorRotate)) * -1;
			motor[PRT_wheelFrontRight] = (DRV_trimChannel(OmniMirrorForward) - DRV_trimChannel(OmniMirrorRotate)) * -1;
			motor[PRT_wheelBackRight]  = (DRV_trimChannel(OmniMirrorForward) - DRV_trimChannel(OmniMirrorRotate)) * -1;
		}

		// Ping
		// Flashes lights on cortex while ping button is down to indicate responsiveness.
		if (vexRT[DRV_config[Ping]] && USR_pingId == -1) {
			USR_pingId = LED_startBlinkTask(Info, Solid);
		} else if (USR_pingId != -1) {
			if (LED_stopBlinkTask(USR_pingId))
				USR_pingId = -1;
		}

		// Override controls
		if (DRV_controllerButtonsDown[Override] == true) {
			// If a combination of the override button and a system button are pressed, toggle that particular system's PID.
			// Toggle the PID then reset motors.
			GUN_enabled = !GUN_enabled;
			PID_enabled = !PID_enabled;
			motor[PRT_gunLeft1] = 0;
			motor[PRT_gunLeft2] = 0;
			motor[PRT_gunRight1] = 0;
			motor[PRT_gunRight2] = 0;


			// Indicate to the DriverControlModule that we have recieved the button press.
			DRV_controllerButtonsDown[Override] = false;
		}

		if (USR_FORCE_AUTON) {
			startTask(autonomous);
			break;
		}
	}
	writeDebugStreamLine("[Mode]: User Control mode disabled.");
}
