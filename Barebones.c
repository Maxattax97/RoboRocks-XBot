#pragma config(Sensor, dgtl1,  PRT_gunLeftQuad, sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  PRT_gunRightQuad, sensorQuadEncoder)
#pragma config(Sensor, dgtl10, PRT_ledR,       sensorLEDtoVCC)
#pragma config(Sensor, dgtl11, PRT_ledY,       sensorLEDtoVCC)
#pragma config(Sensor, dgtl12, PRT_ledG,       sensorLEDtoVCC)
#pragma config(Motor,  port1,           feedLower,     tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           wheelFrontLeft, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           wheelFrontRight, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           wheelBackLeft, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           wheelBackRight, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           gunLeft1,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           gunLeft2,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           gunRight1,     tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           gunRight2,     tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port10,          feedUpper,     tmotorVex393_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//////////////////////////////////////////////////////////////////////////////////////
//                                --= BAREBONES =--                                 //
//////////////////////////////////////////////////////////////////////////////////////
// Barebones is a minimalist programming backup that provides bare minimum          //
// functionality to the robot. In the case of a code emergency, barebones can be    //
// downloaded to ensure the user has secure control of the robot, but will not have //
// intelligent, closed-loop logic that improves driving quality.                    //
//////////////////////////////////////////////////////////////////////////////////////

#pragma platform(VEX)

//Competition Control and Duration Settings
#pragma competitionControl(Competition)
#pragma autonomousDuration(15)
#pragma userControlDuration(120)

//Main competition background code...do not modify!
#include "Vex_Competition_Includes.c"

void pre_auton()
{
	bStopTasksBetweenModes = false;
	SensorValue[PRT_ledR] = 0;
	SensorValue[PRT_ledY] = 0;
	SensorValue[PRT_ledG] = 0;
#if _TARGET == "Emulator"
	startTask(usercontrol);
#endif
}

task autonomous()
{
	// Autonomous Placeholder.
	SensorValue[PRT_ledR] = 0;
	SensorValue[PRT_ledY] = 0;
	SensorValue[PRT_ledG] = 0;
	while (true) {
		SensorValue[PRT_ledR] = 1;
		SensorValue[PRT_ledG] = 0;
		wait1Msec(500);
		SensorValue[PRT_ledR] = 0;
		SensorValue[PRT_ledY] = 1;
		wait1Msec(500);
		SensorValue[PRT_ledY] = 0;
		SensorValue[PRT_ledG] = 1;
		wait1Msec(500);
	}
}

#define LARGE_INC 10
#define SMALL_INC 2
#define TARGET_DEFAULT 80
int targetPower = TARGET_DEFAULT;
int delta = 0;
int power = 0;
bool warmGuns = false;
task guncontrol()
{
	while (true) {
		while (warmGuns) {
			if (power < targetPower && delta <= time1[T1]) {
				power += 1;
				delta = time1[T1] + 100;
			}
			if (power > targetPower) {
				power = targetPower;
			}
			if (power == targetPower) {
				SensorValue[PRT_ledG] = 1;
			} else {
				SensorValue[PRT_ledG] = 0;
			}
			motor[gunLeft1] = power;
			motor[gunLeft2] = power;
			motor[gunRight1] = power;
			motor[gunRight2] = power;
		}
		power = 0;
		delta = 0;
		motor[gunLeft1] = power;
		motor[gunLeft2] = power;
		motor[gunRight1] = power;
		motor[gunRight2] = power;
	}
}

enum Button {warm = 0, plusLarge, plusSmall, minusLarge, minusSmall, reset};
bool gunButtonDown[] = {false, false, false, false, false, false};
bool gunButtonDownPrevious[] = {false, false, false, false, false, false};

task usercontrol()
{
	clearTimer(T1);
	startTask(guncontrol);
	while (true) {
		// Gun Control
		gunButtonDownPrevious[warm] = gunButtonDown[warm];
		if (vexRT[Btn8D] == true) {
			gunButtonDown[warm] = true;
		} else {
			gunButtonDown[warm] = false;
		}

		if (gunButtonDownPrevious[warm] == true && gunButtonDown[warm] == false) {
			warmGuns = !warmGuns;
			gunButtonDown[warm] = false;
		}

		// Gun Incrementers
		gunButtonDownPrevious[plusLarge] = gunButtonDown[plusLarge];
		gunButtonDownPrevious[plusSmall] = gunButtonDown[plusSmall];
		gunButtonDownPrevious[minusLarge] = gunButtonDown[minusLarge];
		gunButtonDownPrevious[minusSmall] = gunButtonDown[minusSmall];
		gunButtonDownPrevious[reset] = gunButtonDown[reset];

		gunButtonDown[plusLarge] = (bool) vexRT[Btn7U];
		gunButtonDown[plusSmall] = (bool) vexRT[Btn7R];
		gunButtonDown[minusLarge] = (bool) vexRT[Btn7D];
		gunButtonDown[minusSmall] = (bool) vexRT[Btn7L];
		gunButtonDown[reset] = (bool) vexRT[Btn8U];

		if (gunButtonDownPrevious[plusLarge] == true && gunButtonDown[plusLarge] == false) {
			targetPower += LARGE_INC;
			gunButtonDown[plusLarge] = false;
		}

		if (gunButtonDownPrevious[plusSmall] == true && gunButtonDown[plusSmall] == false) {
			targetPower += SMALL_INC;
			gunButtonDown[plusSmall] = false;
		}

		if (gunButtonDownPrevious[minusLarge] == true && gunButtonDown[minusLarge] == false) {
			targetPower -= LARGE_INC;
			gunButtonDown[minusLarge] = false;
		}

		if (gunButtonDownPrevious[minusSmall] == true && gunButtonDown[minusSmall] == false) {
			targetPower -= SMALL_INC;
			gunButtonDown[minusSmall] = false;
		}

		if (gunButtonDownPrevious[reset] == true && gunButtonDown[reset] == false) {
			targetPower = TARGET_DEFAULT;
			gunButtonDown[reset] = false;
		}

		if (targetPower > 127)
			targetPower = 127;
		else if (targetPower < 0)
			targetPower = 0;

		// Feed Control
		if (vexRT[Btn5U] == true) {
			motor[feedLower] = 127;
		} else if (vexRT[Btn5D] == true) {
			motor[feedLower] = -127;
		} else {
			motor[feedLower] = 0;
		}

		if (vexRT[Btn6U] == true) {
			motor[feedUpper] = 127;
		} else if (vexRT[Btn6D] == true) {
			motor[feedUpper] = -127;
		} else {
			motor[feedUpper] = 0;
		}

		// Wheel Control
		motor[wheelFrontLeft] = vexRT[Ch3];
		motor[wheelBackLeft] = vexRT[Ch3];
		motor[wheelFrontRight] = vexRT[Ch2];
		motor[wheelBackRight] = vexRT[Ch2];
	}
}
