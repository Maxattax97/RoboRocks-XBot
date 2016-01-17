//////////////////////////////////////////////////////////////////////////////////////
//                                 Autonomous Module                                //
//////////////////////////////////////////////////////////////////////////////////////
// The miscellaneous autonomous functions that allow simpler control over the       //
// robot's individual modules.                                                      //
//////////////////////////////////////////////////////////////////////////////////////

const float AUT_SONAR_INPUT_SCALE = 147.748; // For interpretting sonar distance. (SONAR TICKS / INCH)
const float AUT_SONAR_MAX = AUT_SONAR_INPUT_SCALE * 255; // 21.25 feet.
const float AUT_SONAR_MIN = AUT_SONAR_INPUT_SCALE * 1.1811; // 3 cm.

/*
Sonar must be placed between 3.81 and 7.38 inches above floor to detect low goal.
To remove possibility of detecting single balls, 4.00 to 7.38 inches. (Target: 5.9 inches)
To remove possibility of detecting stacked balls, ~7.27 to 7.38 inches.
*/

void AUT_surge(int power = 127, float time = 0) {
	// To move forward or backward. (Nautical term)
	// Positive is forward, negative is backward.
	motor[PRT_wheelFrontRight] = power;
	motor[PRT_wheelBackRight] = power;
	motor[PRT_wheelFrontLeft] = power;
	motor[PRT_wheelBackLeft] = power;
	if (time != 0) {
		wait1Msec((int)(time * 1000));
		motor[PRT_wheelFrontRight] = 0;
		motor[PRT_wheelBackRight] = 0;
		motor[PRT_wheelFrontLeft] = 0;
		motor[PRT_wheelBackLeft] = 0;
	}
}

void AUT_strafe(int power, float time = 0) {
	// To move left or right without rotation.
	// Positive is to the right, negative is to the left.
	motor[PRT_wheelFrontRight] = -power;
	motor[PRT_wheelBackRight] = power;
	motor[PRT_wheelFrontLeft] = power;
	motor[PRT_wheelBackLeft] = -power;
	if (time != 0) {
		wait1Msec((int)(time * 1000));
		motor[PRT_wheelFrontRight] = 0;
		motor[PRT_wheelBackRight] = 0;
		motor[PRT_wheelFrontLeft] = 0;
		motor[PRT_wheelBackLeft] = 0;
	}
}

void AUT_rotate(int power, float time = 0) {
	// To turn left or right without surge movement.
	// Positive is clockwise, negative is counter clockwise.
	motor[PRT_wheelFrontRight] = -power;
	motor[PRT_wheelBackRight] = -power;
	motor[PRT_wheelFrontLeft] = power;
	motor[PRT_wheelBackLeft] = power;
	if (time != 0) {
		wait1Msec((int)(time * 1000));
		motor[PRT_wheelFrontRight] = 0;
		motor[PRT_wheelBackRight] = 0;
		motor[PRT_wheelFrontLeft] = 0;
		motor[PRT_wheelBackLeft] = 0;
	}
}

void AUT_halt() {
	motor[PRT_wheelFrontRight] = 0;
	motor[PRT_wheelBackRight] = 0;
	motor[PRT_wheelFrontLeft] = 0;
	motor[PRT_wheelBackLeft] = 0;
}

void AUT_feedLower(int power = 127, float time = 0) {
	// To pull inward or push outward from lower belt feed.
	// Positive is inward, negative is outward.
	motor[PRT_feedLower] = power;
	if (time != 0) {
		wait1Msec((int)(time * 1000));
		motor[PRT_feedLower] = 0;
	}
}

void AUT_feedUpper(int power = 127, float time = 0) {
	// To pull inward or push outward from upper belt feed.
	// Positive is inward, negative is outward.
	motor[PRT_feedUpper] = power;
	if (time != 0) {
		wait1Msec((int)(time * 1000));
		motor[PRT_feedUpper] = 0;
	}
}

void AUT_warmGuns() {
	GUN_spool = false;
	GUN_warming = true;
}

void AUT_coolGuns() {
	GUN_warming = false;
	GUN_spool = false;
}

void AUT_spoolGuns() {
	GUN_warming = false;
	GUN_spool = true;
}

void AUT_fireOnce() {
	AUT_warmGuns();
	while (GUN_power < 127) {
		wait1Msec(50);
	}
	AUT_feedLower(127);
	AUT_feedUpper(127);
	wait1Msec(1500);
	AUT_coolGuns();
	motor[PRT_feedLower] = 0;
	motor[PRT_feedUpper] = 0;
}

void AUT_shutDown() {
	AUT_halt();
	AUT_coolGuns();
	motor[PRT_feedLower] = 0;
	motor[PRT_feedUpper] = 0;
}

void AUT_demonstrate() {
	writeDebugStreamLine("[Auton]: Enabling autonomous demonstration procedure.");
	writeDebugStreamLine("[Auton]: Surging forward...");
	AUT_surge(64, 0.5);
	writeDebugStreamLine("[Auton]: Strafing right...");
	AUT_strafe(64, 0.5);
	writeDebugStreamLine("[Auton]: Surging backward...");
	AUT_surge(-64, 0.5);
	writeDebugStreamLine("[Auton]: Strafing left...");
	AUT_strafe(-64, 0.5);
	writeDebugStreamLine("[Auton]: Rotating right...");
	AUT_rotate(64, 0.5);
	writeDebugStreamLine("[Auton]: Strafing left...");
	AUT_rotate(-64, 0.5);
	writeDebugStreamLine("[Auton]: Warming gun...");
	AUT_fireOnce();
	writeDebugStreamLine("[Auton]: Gun fired!");
	AUT_shutDown();
	writeDebugStreamLine("[Auton]: Shutting down...");
	writeDebugStreamLine("[Auton]: Autonomous demonstration procedure complete.");
}
