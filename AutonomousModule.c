//////////////////////////////////////////////////////////////////////////////////////
//                                 Autonomous Module                                //
//////////////////////////////////////////////////////////////////////////////////////
// The miscellaneous autonomous functions that allow simpler control over the       //
// robot's individual modules.                                                      //
//////////////////////////////////////////////////////////////////////////////////////

const float AUT_target = TRJ_angularSpeedAtRange(13);

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
	if (GUN_enabled) {
		GUN_spool = false;
		GUN_warming = true;
	} else if (PID_enabled) {
		PID_target[Left] = AUT_target;
		PID_target[Right] = AUT_target;
	}
}

void AUT_coolGuns() {
	if (GUN_enabled) {
		GUN_warming = false;
		GUN_spool = false;
	} else if (PID_enabled) {
		PID_target[Left] = 0;
		PID_target[Right] = 0;
	}
}
/*
void AUT_spoolGuns() {
	GUN_warming = false;
	GUN_spool = true;
}
*/
void AUT_fireOnce() {
	AUT_warmGuns();
	if (GUN_enabled) {
		while (GUN_power < 127) {
			wait1Msec(50);
		}
		AUT_feedLower(127);
		AUT_feedUpper(127);
		wait1Msec(1500);
		AUT_coolGuns();
		motor[PRT_feedLower] = 0;
		motor[PRT_feedUpper] = 0;
	} else if (PID_enabled) {
		while (!PID_ready) {
			wait1Msec(50);
		}
		AUT_feedLower(127);
		AUT_feedUpper(127);
		wait1Msec(1500);
		AUT_coolGuns();
		motor[PRT_feedLower] = 0;
		motor[PRT_feedUpper] = 0;
	}
}

void AUT_shutDown() {
	AUT_halt();
	AUT_coolGuns();
	motor[PRT_feedLower] = 0;
	motor[PRT_feedUpper] = 0;
}
/*
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
*/
