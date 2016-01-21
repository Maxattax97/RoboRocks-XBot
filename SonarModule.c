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

/*
Sonar must be placed between 3.81 and 7.38 inches above floor to detect low goal.
To remove possibility of detecting single balls, 4.00 to 7.38 inches. (Target: 5.9 inches)
To remove possibility of detecting stacked balls, ~7.27 to 7.38 inches.
*/

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
