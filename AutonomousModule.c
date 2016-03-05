//////////////////////////////////////////////////////////////////////////////////////
//                                 Autonomous Module                                //
//////////////////////////////////////////////////////////////////////////////////////
// The miscellaneous autonomous functions that allow simpler control over the       //
// robot's individual modules.                                                      //
//////////////////////////////////////////////////////////////////////////////////////

const float AUT_target = TRJ_angularSpeedAtRange(13);
const float AUT_SHORT_SPEED = 1250;
int AUT_ballsFired = 0;

task AUT_countFiredBalls() {
	AUT_ballsFired = 0;
	writeDebugStreamLine("[Auton]: Counting fired balls...");
	while (true) {
		if (PID_ballFired == true) {
			AUT_ballsFired++;
			writeDebugStreamLine("[Auton]: Ball %i fired!", AUT_ballsFired);
			/*PID_ballFired = false;
			while (true) {
				wait1Msec(1000 / PID_SPEED_INTERVALS_PER_SECOND);
				if (PID_ballFired == true) {
					PID_ballFired = false; // Cancel event.
				} else {
					break; // Ball is "fully fired".
				}
			}*/
			wait1Msec(200);
			PID_ballFired = false;
		} else {
			wait1Msec(1000 / PID_SPEED_INTERVALS_PER_SECOND * 2);
		}
	}
}

void AUT_surge(int power = 127, float time = 0) {
	// To move forward or backward. (Nautical term)
	// Positive is forward, negative is backward.
	motor[PRT_wheelsLeft] = power;
	motor[PRT_wheelsRight] = power;
	/*motor[PRT_wheelFrontRight] = power;
	motor[PRT_wheelBackRight] = power;
	motor[PRT_wheelFrontLeft] = power;
	motor[PRT_wheelBackLeft] = power;*/
	if (time != 0) {
		wait1Msec((int)(time * 1000));
		motor[PRT_wheelsLeft] = power;
		motor[PRT_wheelsRight] = power;
		/*motor[PRT_wheelFrontRight] = 0;
		motor[PRT_wheelBackRight] = 0;
		motor[PRT_wheelFrontLeft] = 0;
		motor[PRT_wheelBackLeft] = 0;*/
	}
}

/*
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
}*/

void AUT_rotate(int power, float time = 0) {
	// To turn left or right without surge movement.
	// Positive is clockwise, negative is counter clockwise.
	motor[PRT_wheelsLeft] = power;
	motor[PRT_wheelsRight] = -power;
	/*motor[PRT_wheelFrontRight] = -power;
	motor[PRT_wheelBackRight] = -power;
	motor[PRT_wheelFrontLeft] = power;
	motor[PRT_wheelBackLeft] = power;*/
	if (time != 0) {
		wait1Msec((int)(time * 1000));
		motor[PRT_wheelsLeft] = 0;
		motor[PRT_wheelsRight] = 0;
		/*motor[PRT_wheelFrontRight] = 0;
		motor[PRT_wheelBackRight] = 0;
		motor[PRT_wheelFrontLeft] = 0;
		motor[PRT_wheelBackLeft] = 0;*/
	}
}

void AUT_halt() {
	motor[PRT_wheelsLeft] = 0;
	motor[PRT_wheelsRight] = 0;
	/*motor[PRT_wheelFrontRight] = 0;
	motor[PRT_wheelBackRight] = 0;
	motor[PRT_wheelFrontLeft] = 0;
	motor[PRT_wheelBackLeft] = 0;*/
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

// Align the robot perpendicular to a surface using sonar. Both values must be within tolerance of each other.
// If the robot needs to be slightly off of perpendicular, offset the right side by a percentage to reach a
// desired angle.
bool AUT_alignWithSonar(float left, float right, float tolerance = 0.5, float rightPercentage = 1.0) {
	if (left != SNR_INVALID && right != SNR_INVALID) {
			if (abs((rightPercentage * left) - right) <= tolerance) {
				AUT_halt();
				return true; // Return true to indicate now aligned.
			} else if ((rightPercentage * left) > right) {
				AUT_rotate(-48); // Closer to right. Rotate counter-clockwise.
			} else if (right > (left * rightPercentage)) {
				AUT_rotate(48); // Closer to left. Rotate clockwise.
			}
	}
	// Don't move anything. Wait for input.
	return false;
}

bool AUT_distanceWithSonar(float left, float right, float target, float tolerance = 0.5) {
	if (SNR_distanceInchesLeft != SNR_INVALID && SNR_distanceInchesRight != SNR_INVALID) {
		float average = (SNR_distanceInchesLeft + SNR_distanceInchesRight) / 2;
		if (average < target - tolerance) {
			AUT_surge(28);
		} else if (average > target + tolerance) {
			AUT_surge(-28);
		} else {
			AUT_halt();
			return true;
		}
		wait1Msec(50);
	}
	// Don't move anything. Wait for input.
	return false;
}

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

void AUT_demonstrate() {
	writeDebugStreamLine("[Auton]: Enabling autonomous demonstration procedure.");
	writeDebugStreamLine("[Auton]: Surging forward...");
	AUT_surge(64, 0.5);
	writeDebugStreamLine("[Auton]: Strafing right...");
	//AUT_strafe(64, 0.5);
	writeDebugStreamLine("[Auton]: Surging backward...");
	AUT_surge(-64, 0.5);
	writeDebugStreamLine("[Auton]: Strafing left...");
	//AUT_strafe(-64, 0.5);
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
