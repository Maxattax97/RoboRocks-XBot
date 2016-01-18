//////////////////////////////////////////////////////////////////////////////////////
//                                 PID Loop Module                                  //
//////////////////////////////////////////////////////////////////////////////////////
// The PID loop is an error solving algorithm. A target is set, then the algorithm  //
// evaluates its actual position in relation to its target. It then decides how to  //
// best reach that target by direction and distance. It then applies motors         //
// proportional to the error value to meet its target. This system adds a	          //
// significant degree of intelligence to the robot. It allows the robot to counter  //
// objects' weight, the system's own weight, and other outside forces. It is        //
// currently applied across three seperate subsystems: the main lift, the claw lift,//
// and the claw wrist.                                                              //
//////////////////////////////////////////////////////////////////////////////////////
// NOTE: MUCH OF THIS MODULE IS CURRENTLY DEPRECATED. DO NOT USE!                   //
//////////////////////////////////////////////////////////////////////////////////////


// Enum to access the PID's two halves.
enum PID_Side {Left = 0, Right};

bool PID_enabled = true; // Whether or not the PID is enabled for the system.
const bool PID_INTEGRAL_ENABLED = true;
const bool PID_DERIVATIVE_ENABLED = true;
const int PID_POWER_MAX = 127; // Maximum and minimum amount of power for the PID to apply.
const int PID_POWER_MIN = 0;
const int PID_INTEGRAL_LIMIT = 30000; // Threshold for integral values.
const int PID_INTERVALS_PER_SECOND = 42; // Rate in hertz that the PID evaluates at.
const int PID_SPEED_INTERVALS_PER_SECOND = 42; // Rate in hertz that the speed sampler evaluates at.
const int PID_QUAD_MAX = 32767;
const int PID_QUAD_MIN = -32768;

const float PID_INPUT_SCALE = 120; // Set this to be ENCODER TICKS / REVOLUTION.
const float PID_INPUT_MIRROR[] = {1.0, -1.0}; // Flips sensor direction. Encoders read clockwise as increasing.
float PID_SIGNAL_GENERATOR_SLOPE = 250; // Slope value for "horse and carrot".
const int PID_SYSTEM_HYSTERESIS = 8; // Solves for motor jogging without breaking PID.
const int PID_DEADBAND = 30; // Amount of power to apply to turn wheels minimally.
const float PID_DEADBAND_THRESHOLD = 120; // Must be moving under this speed to apply deadband.
const float PID_targetMax = 2100; // Maximum speed in RPM.

float PID_target[] = {0, 0}; // For setting the target speed. Do not touch.
float PID_actual[] = {0, 0}; // For detecting actual speed. Do not touch.
float PID_speedDeltaStart[] = {0, 0}; // For calculating speed of flywheel.
float PID_speedDelta[] = {0, 0};
float PID_deltaStart[] = {0, 0}; // For calculating PID timing.
float PID_delta[] = {0, 0};
float PID_lastQuad[] = {0, 0};
const int PID_HISTORY_LENGTH = 5;
float PID_actualHistory[2][PID_HISTORY_LENGTH];
int PID_historyIndex[] = {0, 0};

float PID_filteredTarget[] = {0, 0}; // Target with signal generator applied (horse and carrot).
float PID_power[] = {0, 0}; // Motor power requested to acquire target speed.
float PID_error[] = {0, 0}; // Difference between actual and target.
float PID_lastError[] = {0, 0}; // For calculation of derivative.
float PID_integral[] = {0, 0}; // For accumulation of error.
float PID_derivative[] = {0, 0}; // For prediction of incoming error.
float PID_lastTarget[] = {0, 0}; // For signal generator calculation.
float PID_lastActual[] = {0, 0}; // For calculation of derivative.

const int PID_OVERLOAD_THRESHOLD = 300; // If the PID is behind this much, activates overload blink task.
short PID_blinkIdDisabled[] = {NULL, NULL}; // Placeholders for blink task ID's.
short PID_blinkIdOverloaded[] = {NULL, NULL};

// PID Loop constants for Proportional, Integral, and Derivative (Items in the acronym PID).
const float PID_KU = 1.333;
const float PID_TU = 0.018;
float PID_KP = 0.5;
float PID_KI = 0.01;
float PID_KD = 0;
/*
Ziegler-Nichols Method for Tuning PID Loops

	 1. Adjust Kp until oscillates evenly (oscillation does not gain or lose).
	 2. Ku = current Kp.
	 3. Period of oscillation = Tu.
	 4. Follow table:

	 Type |   Kp   |    Ki    |   Kd
	    P | 0.50Ku |        - |      -
	   PI | 0.45Ku |   Tu/1.2 |      -
	   PD | 0.80Ku |        - |   Tu/8
	  PID | 0.60Ku |     Tu/2 |   Tu/8
Tyreus-Luyben
     PI | Ku/3.2 |    2.2Tu |      -
    PID | Ku/2.2 |    2.2Tu | Tu/6.3
*/

// TODO: Add info led for gun ready.

// Internal PID variables, not meant to be touched by outside modules. Also allows for debugging.
/*enum PID_Internal {filteredTarget, power, error, lastError, integral, derivative, lastTarget, lastActual};
float PID_internals[2][8];*/

// A single PID cycle.
void PID_cycle(PID_Side index) {
	if (PID_enabled) {
		PID_delta[index] = (((float)time1[T1]) / 1000) - PID_deltaStart[index];
		// Cap the target so it doesn't get too crazy.
		if (PID_target[index] > PID_targetMax) {
			PID_target[index] = PID_targetMax;
		} else if (PID_target[index] < 0) {
			PID_target[index] = 0;
		}

		// Target filter using a signal generator (horse and carrot method).
		// The horse and carrot method is a way of visualizing the movement of the PID's actuator
		// in relation to the target's movement. The "carrot" (target) is gradually dragged to another location
		// for the "horse" (actuator) to move to. The carrot does not teleport from one place to another.
		// In this case, however, we WILL teleport the carrot. But only to a lower place than before...
		if (PID_target[index] <= PID_lastTarget[index]) {
			PID_filteredTarget[index] = PID_target[index];
		} else {
			PID_filteredTarget[index] += PID_SIGNAL_GENERATOR_SLOPE * PID_delta[index];
		}
		PID_lastTarget[index] = PID_filteredTarget[index];

		// Calculate the error.
		PID_error[index] = PID_actual[index] - PID_filteredTarget[index];

		if (PID_INTEGRAL_ENABLED && PID_KI != 0) {
			// Accumulate error to fill the gap.
				PID_integral[index] += PID_error[index];
		} else {
			PID_integral[index] = 0;
		}

		if (PID_integral[index] > PID_INTEGRAL_LIMIT) {
			PID_integral[index] = PID_INTEGRAL_LIMIT;
		} else if (PID_integral[index] < -1 * PID_INTEGRAL_LIMIT) {
			PID_integral[index] = -1 * PID_INTEGRAL_LIMIT;
		}

		if (PID_DERIVATIVE_ENABLED && PID_KD != 0) {
			// Predict future change via derivative.
			PID_derivative[index] = (PID_error[index] - PID_lastError[index]);
			PID_lastError[index] = PID_error[index];
		} else {
			PID_derivative[index] = 0;
		}

		// Calculate the power necessary to correct the found error.
		// Negations are to keep error and power values in a logical context.
		PID_power[index] = -1 * ((PID_KP * PID_error[index])
				+ (PID_KI * PID_integral[index])
				+ (PID_KD * PID_derivative[index]));

		// Limit the power sent to motors. (Can't exceed the byte)
		if (PID_power[index] > PID_POWER_MAX) {
			PID_power[index] = PID_POWER_MAX;
		} else if (PID_power[index] < PID_POWER_MIN) {
			PID_power[index] = PID_POWER_MIN;
		}

		// If the power level is below the deadband, increase it so that the
		// motor is actually moving toward its goal. Only apply deadband if below the
		// deadband threshold.
		// Disabled for tuning purposes.
		if (PID_power[index] > 0 && PID_power[index] < PID_DEADBAND
				&& PID_DEADBAND != 0 && PID_power[index] != 0
				&& PID_actual[index] < PID_DEADBAND_THRESHOLD) {
			PID_power[index] = PID_DEADBAND;
		}

		// Set the motor power value.
		// Type cast to keep from wrongly setting the motors.
		if (index == Left) {
			motor[PRT_gunLeft1] = (int) PID_power[index];
			motor[PRT_gunLeft2] = (int) PID_power[index];
		} else if (index == Right) {
			motor[PRT_gunRight1] = (int) PID_power[index];
			motor[PRT_gunRight2] = (int) PID_power[index];
		}

		// Indicate that PID is struggling to meet its target.
		if (PID_error[index] <= -1 * PID_OVERLOAD_THRESHOLD) {
			if (PID_blinkIdOverloaded[index] == NULL) {
				PID_blinkIdOverloaded[index] = LED_startBlinkTask(Severe, Solid);
			}
		} else if (PID_blinkIdOverloaded[index] != NULL) {
			// If we are not struggling, disable the overload blink task.
			LED_stopBlinkTask(PID_blinkIdOverloaded[index]);
			PID_blinkIdOverloaded[index] = NULL;
		}

		// Shut the disabled blink task off.
		if (PID_blinkIdDisabled[index] != NULL) {
			LED_stopBlinkTask(PID_blinkIdDisabled[index]);
			PID_blinkIdDisabled[index] = NULL;
		}
		PID_deltaStart[index] = ((float)time1[T1]) / 1000;
	} else {
		// Reset the variables.
		PID_error[index] = 0;
		PID_lastError[index] = 0;
		PID_integral[index] = 0;
		PID_derivative[index] = 0;
		PID_power[index] = 0;
		// Reset the target so it doesn't jump when it is re-enabled.
		PID_target[index] = 0;
		// Enable the disabled blink task.
		if (PID_blinkIdDisabled[index] == NULL) {
			if (index == Left) {
				PID_blinkIdDisabled[index] = LED_startBlinkTask(Warning, Slow);
			} else if (index == Right) {
				PID_blinkIdDisabled[index] = LED_startBlinkTask(Warning, Medium);
			}
			if (PID_blinkIdOverloaded[index] != NULL) {
				LED_stopBlinkTask(PID_blinkIdOverloaded[index]);
				PID_blinkIdOverloaded[index] = NULL;
			}
		}
		// Be sure to stop the motor as well. (Externally once, rather than many times)
	}
}

task PID_speedLoop() {
	writeDebugStreamLine("[PID]: Speed sampler started.");
	while (true) {
		for (short index = Left; index < Right + 1; index++) {
			// Determine how much time has elapsed (in minutes for RPM).
			PID_speedDelta[index] = ((((float)time1[T1]) / 1000) / 60) - PID_speedDeltaStart[index];
			if (PID_speedDelta[index] > 0) {
				// Grab the actual sensor value, then scale it (if necessary).
				//writeDebugStreamLine("quad: %i, scale: %f, delta: %f, mirror: %d", SensorValue[PRT_gunLeftQuad], PID_INPUT_SCALE, PID_delta[index], PID_INPUT_MIRROR[index])
				PID_lastActual[index] = PID_actual[index];
				int ticks = 0;
				if (index == Left) {
					ticks = (PID_INPUT_MIRROR[index] * SensorValue[PRT_gunLeftQuad]) - PID_lastQuad[index];
					PID_lastQuad[index] = (PID_INPUT_MIRROR[index] * SensorValue[PRT_gunLeftQuad]);
				}
				else if (index == Right) {
					ticks = (PID_INPUT_MIRROR[index] * SensorValue[PRT_gunRightQuad]) - PID_lastQuad[index];
					PID_lastQuad[index] = (PID_INPUT_MIRROR[index] * SensorValue[PRT_gunRightQuad]);
				}

				// Retrieve speed in RPM.
				if (ticks < 0) {
					ticks += (-1 * PID_QUAD_MIN) + PID_QUAD_MAX;
				}
				PID_actualHistory[index][PID_historyIndex[index]] = (ticks / PID_INPUT_SCALE) / PID_speedDelta[index];
				PID_historyIndex[index]++;
				if (PID_historyIndex[index] == PID_HISTORY_LENGTH) {
					PID_historyIndex[index] = 0;
				}
				float sum = 0;
				for (int i = 0; i < PID_HISTORY_LENGTH; i++) {
					sum += PID_actualHistory[index][i];
				}
				PID_actual[index] = sum / PID_HISTORY_LENGTH;
				// Record the time between speed measurements.
				PID_speedDeltaStart[index] = (((float)time1[T1]) / 1000) / 60;
			}
		}
		wait1Msec(1000 / PID_SPEED_INTERVALS_PER_SECOND);
	}
}

task PID_blinkReadiness() {
	float totalReadiness = 0; // Closer to 0 means more ready.
	float goodEnoughThreshold = 0.05;
	while (true) {
		totalReadiness = 0;
		if (PID_target[Left] != 0) {
			totalReadiness += abs(1 - (PID_actual[Left] / PID_target[Left]));
		}
		if (PID_target[Right] != 0) {
			totalReadiness += abs(1 - (PID_actual[Right] / PID_target[Right]));
		}
		if (totalReadiness <= goodEnoughThreshold) {
			SensorValue[PRT_ledGun] = 1;
			wait1Msec(50);
		} else {
			SensorValue[PRT_ledGun] = 0;
			wait1Msec(LIMIT(1, 1000, 1000 * totalReadiness));
			SensorValue[PRT_ledGun] = 1;
			wait1Msec(50);
		}
	}
}

bool PID_capture = false;
bool PID_useSonar = false;

task PID_controller() {
	//bool batLowRetrieved = false;
	PID_deltaStart[Left] = ((float)time1[T1]) / 1000;
	PID_deltaStart[Right] = ((float)time1[T1]) / 1000;
	wait1Msec(1000 / PID_INTERVALS_PER_SECOND);
	startTask(PID_speedLoop);
	startTask(PID_blinkReadiness);
	while (true) {
		// Run each individual cycle per system.
		if (PID_useSonar == true && SNR_distanceInches >= 12*7 && SNR_distanceInches <= 12*15) {
			PID_target[Left] = SNR_angularSpeedAtRange(SNR_distanceInches / 12);
			PID_target[Right] = PID_target[Left];
		}
		PID_cycle(Left);
		PID_cycle(Right);
		if (PID_capture == true && PID_power > 0)
			writeDebugStreamLine("%f, %f, %f, %f, %f, %f, %f, %f, %f",
				((float)time1[T1]) / 1000, PID_filteredTarget[Left], PID_actual[Left], PID_power[Left], PID_error[Left],
				PID_filteredTarget[Right], PID_actual[Right], PID_power[Right], PID_error[Right]);
		/*// Check the battery to initiate emergency PID shut off.
		if (batLowRetrieved == false && BAT_low == true) {
			batLowRetrieved = true;
			writeDebugStreamLine("[PID]: Activating emergency PID shut off to conserve power...");
			// The user can re-enable these with PID override.
			PID_enabled[MainLift] = false;
			PID_enabled[ClawLift] = false;
			PID_enabled[ClawWrist] = false;
		} else if (BAT_low == false) {
			batLowRetrieved = false;
		}*/

		// Repeat at defined delta time, in Hertz.
		wait1Msec(1000 / PID_INTERVALS_PER_SECOND);
	}
}
