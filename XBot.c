#pragma config(Sensor, dgtl1,  PRT_gunLeftQuad, sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  PRT_gunRightQuad, sensorQuadEncoder)
#pragma config(Sensor, dgtl10, PRT_ledR,       sensorLEDtoVCC)
#pragma config(Sensor, dgtl11, PRT_ledY,       sensorLEDtoVCC)
#pragma config(Sensor, dgtl12, PRT_ledG,       sensorLEDtoVCC)
#pragma config(Motor,  port1,           PRT_feedLower, tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           PRT_wheelFrontLeft, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           PRT_wheelFrontRight, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           PRT_wheelBackLeft, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           PRT_wheelBackRight, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           PRT_gunLeft1,  tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           PRT_gunLeft2,  tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           PRT_gunRight1, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           PRT_gunRight2, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port10,          PRT_feedUpper, tmotorVex393_HBridge, openLoop)
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

#if (_TARGET == EMULATOR)
#warning "###!!!### RUNNING IN EMULATOR MODE ###!!!###"
#define EMULATING 1
#else
#define EMULATING 0
#endif

#if (DEBUG == 1)
#warning "###!!!### RUNNING IN DEBUG MODE ###!!!###"
#endif

//////////////////
// DEPENDENCIES //
//////////////////

#include "Utilities.c"
#include "BlinkModule.c"
#include "BatteryModule.c"
#include "DriverControlModule.c"
#include "GunModule.c"
#include "PIDLoopModule.c"
#include "AutonomousModule.c"

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
	SensorValue[PRT_gunLeftQuad] = 0;
	SensorValue[PRT_gunRightQuad] = 0;

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
	short id = LED_startBlinkTask(Info, Slow);
	writeDebugStreamLine("[Mode]: Autonomous mode enabled!");
	//AUT_demonstrate();
	writeDebugStreamLine("[Auton]: Warming guns...");
	AUT_warmGuns();
	while (GUN_power < GUN_maxMotorPower) { // Max power changed
		wait1Msec(50);
	}
	//wait1Msec(1000);
	writeDebugStreamLine("[Auton]: Guns warmed. Firing ball 1.");
	//AUT_feedUpper(127, 1); // Fire first ball.
	motor[PRT_feedUpper] = 127;
	wait1Msec(500);
	motor[PRT_feedUpper] = 0;
	wait1Msec(2000);
	for (int i = 0; i < 3; i++) {
			motor[PRT_feedLower] = 127;
			motor[PRT_feedUpper] = 127;
			//AUT_feedLower(127); // Enable feeds.
			//AUT_feedUpper(127);
			writeDebugStreamLine("[Auton]: Firing ball %i.", i + 2);
			//AUT_surge(127, 1); // Move forward onto ball.
			wait1Msec(2500);
			writeDebugStreamLine("[Auton]: Ball fired.");
			//AUT_feedLower(0); // Disable feeds.
			//AUT_feedUpper(0);
			motor[PRT_feedLower] = 0;
			motor[PRT_feedUpper] = 0;
			//AUT_surge(-127, 1); // Move back into place.
			writeDebugStreamLine("[Auton]: Allowing guns to adjust.");
			wait1Msec(2000);
	}
	writeDebugStreamLine("[Auton]: Shutting down.");
	AUT_shutDown();
	writeDebugStreamLine("[Mode]: Autonomous mode disabled.");
	LED_stopBlinkTask(id);
}

///////////////////////////
//// USER CONTROL MODE ////
///////////////////////////

short USR_pingId = -1;

// For manual, computer-based value editing.
bool USR_OVERRIDE_USER_CONTROL = false;
#if (DEBUG == 1)
USR_OVERRIDE_USER_CONTROL = true;
#endif

task usercontrol()
{
	writeDebugStreamLine("[Mode]: User Control mode enabled!");
	// Continually check for user input.
	while (!USR_OVERRIDE_USER_CONTROL) {
		// Feed Control
		if (DRV_config[FeedLowerIn] != UNASSIGNED && vexRT[DRV_config[FeedLowerIn]] == true) {
			motor[PRT_feedLower] = 127;
		} else if (DRV_config[FeedLowerOut] != UNASSIGNED && vexRT[DRV_config[FeedLowerOut]] == true) {
			motor[PRT_feedLower] = -127;
		} else {
			motor[PRT_feedLower] = 0;
		}
		if (DRV_config[FeedUpperIn] != UNASSIGNED && vexRT[DRV_config[FeedUpperIn]] == true) {
			motor[PRT_feedUpper] = 127;
		} else if (DRV_config[FeedUpperOut] != UNASSIGNED && vexRT[DRV_config[FeedUpperOut]] == true) {
			motor[PRT_feedUpper] = -127;
		} else {
			motor[PRT_feedUpper] = 0;
		}

		// Gun Control
		if (DRV_controllerButtonsDown[GunIncrement] == true) {
			GUN_maxMotorPower += GUN_LARGE_INCREMENT;
			DRV_controllerButtonsDown[GunIncrement] = false;
		} else if (DRV_controllerButtonsDown[GunDecrement] == true) {
			GUN_maxMotorPower -= GUN_LARGE_INCREMENT;
			DRV_controllerButtonsDown[GunDecrement] = false;
		}

		if (DRV_controllerButtonsDown[GunSmallIncrement] == true) {
			GUN_maxMotorPower += GUN_SMALL_INCREMENT;
			DRV_controllerButtonsDown[GunSmallIncrement] = false;
		} else if (DRV_controllerButtonsDown[GunSmallDecrement] == true) {
			GUN_maxMotorPower -= GUN_SMALL_INCREMENT;
			DRV_controllerButtonsDown[GunSmallDecrement] = false;
		}

		if (DRV_controllerButtonsDown[GunWarm] == true) {
			GUN_maxMotorPower = GUN_DEFAULT_POWER;
			GUN_warming = !GUN_warming;
			DRV_controllerButtonsDown[GunWarm] = false;
		}/* else if (DRV_controllerButtonsDown[GunSpool] == true) {
			GUN_spool = !GUN_spool;
			DRV_controllerButtonsDown[GunSpool] = false;
		}*/

		// Wheel Control
		// Setup the wheel reversal.
		int reverseMultiplier = 1;
		// TODO: THIS IS WRONG. AFTER ONE LOOP, WILL RESET. EVENT IS CONSUMED!!!
		if (DRV_controllerButtonsDown[ToggleMirror] == true) {
			reverseMultiplier *= -1; // Flip the multiplier's sign.
			DRV_controllerButtonsDown[ToggleMirror] = false; // Consume event.
		}

		if (DRV_config[OmniLeft] != UNASSIGNED && DRV_config[OmniRight] != UNASSIGNED) {
			// 2 channel (tank) drive for OMNI WHEELS enabled.
			motor[PRT_wheelFrontLeft]  = DRV_trimChannel(OmniLeft)  * reverseMultiplier;
			motor[PRT_wheelBackLeft]   = DRV_trimChannel(OmniLeft)  * reverseMultiplier;
			motor[PRT_wheelFrontRight] = DRV_trimChannel(OmniRight) * reverseMultiplier;
			motor[PRT_wheelBackRight]  = DRV_trimChannel(OmniRight) * reverseMultiplier;
		} else if (DRV_config[OmniForward] != UNASSIGNED && DRV_config[OmniRotate] != UNASSIGNED) {
			// 2 channel (smart) drive for OMNI WHEELS enabled.
			motor[PRT_wheelFrontLeft]  = (DRV_trimChannel(OmniForward) + DRV_trimChannel(OmniRotate)) * reverseMultiplier;
			motor[PRT_wheelBackLeft]   = (DRV_trimChannel(OmniForward) + DRV_trimChannel(OmniRotate)) * reverseMultiplier;
			motor[PRT_wheelFrontRight] = (DRV_trimChannel(OmniForward) - DRV_trimChannel(OmniRotate)) * reverseMultiplier;
			motor[PRT_wheelBackRight]  = (DRV_trimChannel(OmniForward) - DRV_trimChannel(OmniRotate)) * reverseMultiplier;
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

		if (DRV_config[OmniMirrorForward] != UNASSIGNED && DRV_config[OmniMirrorRotate] != UNASSIGNED) {
			// 2 channel (smart) MIRRORED drive for OMNI WHEELS enabled.
			motor[PRT_wheelFrontLeft]  = (DRV_trimChannel(OmniForward) + DRV_trimChannel(OmniRotate)) * -1;
			motor[PRT_wheelBackLeft]   = (DRV_trimChannel(OmniForward) + DRV_trimChannel(OmniRotate)) * -1;
			motor[PRT_wheelFrontRight] = (DRV_trimChannel(OmniForward) - DRV_trimChannel(OmniRotate)) * -1;
			motor[PRT_wheelBackRight]  = (DRV_trimChannel(OmniForward) - DRV_trimChannel(OmniRotate)) * -1;
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
			/*if (vexRT[DRV_config[LiftUp]] == true || vexRT[DRV_config[LiftDown]] == true
					|| DRV_trimChannel(LiftJoy) != 0) {
				// Toggle the main lift system then reset motors.
				PID_enabled[MainLift] = !PID_enabled[MainLift];
				motor[PRT_leftLiftMotors] = 0;
				motor[PRT_rightLiftMotors] = 0;
				// Indicate to the DriverControlModule that we have recieved the button press.

				DRV_controllerButtonsDown[Override] = false;
			}*/
			// No systems were designated. Exit WITHOUT accepting button down event.
		}
	}
	writeDebugStreamLine("[Mode]: User Control mode disabled.");
}
