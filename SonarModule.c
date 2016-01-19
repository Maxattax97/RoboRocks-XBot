//////////////////////////////////////////////////////////////////////////////////////
//                                   Sonar Module                                   //
//////////////////////////////////////////////////////////////////////////////////////
// This module provides distance data to other modules, allows locking of targets,  //
// and calculation of required RPM.                                                 //
//////////////////////////////////////////////////////////////////////////////////////

const float SNR_INPUT_SCALE = 147.748; // For interpretting sonar distance. (SONAR TICKS / INCH)
const float SNR_MAX = 240; // 20 feet.
const float SNR_MIN = 2; // 2 inches.
const signed char SNR_INVALID = -88; // Designates a value that is too close or too long to be accurate.
float SNR_distanceInches = 0; // Distance from sonar to focused object in inches.
const int SNR_FREQUENCY = 10; // In hertz, how fast sonar calculations will be made.

const float SNR_GUN_OFFSET = 1.5; // Distance from front of gun to front of sonar in inches.
const float SNR_NET_OFFSET = 0; //21.696; // Distance from the front of the low goal to the middle of the net opening.
const float SNR_GUN_ANGLE = 23; // Inclination in degrees of gun.
const float SNR_GRAVITY = 386.088; // Acceleration in inches/s^2 of gravity.
const float SNR_GUN_HEIGHT = 13; // Height in inches of the gun from the ground.
const float SNR_NET_HEIGHT = 36.16; // Lower height of net opening in inches.
const float SNR_BALL_RADIUS = 2; // Radius of a game ball in inches.
const float SNR_IPS_TO_RPM = 4.774648294; // Constant to multiply by inches/s to acquire rev/min.
const float SNR_RANGE_MIN = 67.3025796; // Range that is too close for a successful shot.
const float SNR_RANGE_MAX = 278.022252; // Range that is too far for a successful shot.

/*
Sonar must be placed between 3.81 and 7.38 inches above floor to detect low goal.
To remove possibility of detecting single balls, 4.00 to 7.38 inches. (Target: 5.9 inches)
To remove possibility of detecting stacked balls, ~7.27 to 7.38 inches.
*/

// Determine whether or not the range is acceptable.
bool SNR_validRange(float range) {
	if (range != SNR_INVALID && range >= SNR_RANGE_MIN && range <= SNR_RANGE_MAX) {
		return true;
	}
	return false;
}

// Input range in inches, output angular velocity in RPM.
float SNR_angularSpeedAtRange(float range) {
	if (SNR_validRange(range)) {
		return SNR_IPS_TO_RPM * ((1 / cosDegrees(SNR_GUN_ANGLE)) *
			sqrt((0.5 * SNR_GRAVITY * pow(range, 2))
			/ (range * tanDegrees(SNR_GUN_ANGLE) - (SNR_NET_HEIGHT - SNR_GUN_HEIGHT - 2*SNR_BALL_RADIUS))));
	}
	return SNR_INVALID;
}

task SNR_update() {
	//writeDebugStreamLine("[Sonar]: The sonar sensor is currently reading %1.3f inches.", ((float) SensorValue[PRT_sonar]) * SNR_INPUT_SCALE);
	while (true) {
		float dist = ((float) SensorValue[PRT_sonar]) / SNR_INPUT_SCALE;
		if (dist <= SNR_MIN || dist >= SNR_MAX) {
			dist = SNR_INVALID;
		}
		SNR_distanceInches = dist;
		wait1Msec(1000 / SNR_FREQUENCY);
	}
}
