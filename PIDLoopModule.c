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
const bool PID_INTEGRAL_ENABLED = false;
const bool PID_DERIVATIVE_ENABLED = false;
const int PID_POWER_MAX = 127; // Maximum and minimum amount of power for the PID to apply.
const int PID_POWER_MIN = -127;
const int PID_INTEGRAL_LIMIT = 50; // Threshold for integral values.
const int PID_INTERVALS_PER_SECOND = 15; // Rate in hertz that the PID evaluates at.

float PID_INPUT_SCALE = 0.03333333333333333333333333333333; // Set this to be ENCODER TICKS / REVOLUTION. [(1/90) * 3]
float PID_INPUT_MIRROR[] = {1.0, -1.0}; // Flips sensor direction. Encoders read clockwise as increasing.
float PID_SIGNAL_GENERATOR_MAX = 0.99; // Percentage value for "horse and carrot".
const int PID_SYSTEM_HYSTERESIS = 8; // Solves for motor jogging without breaking PID.
const float PID_targetMax = 2100; // Maximum speed in RPM.

float PID_target[] = {0, 0}; // For setting the target speed. Do not touch.
float PID_actual[] = {0, 0}; // For detecting actual speed. Do not touch.
float PID_deltaStart[] = {0, 0}; // For calculating speed of flywheel.
float PID_delta[] = {0, 0};

float PID_filteredTarget[] = {0, 0}; // Target with signal generator applied (horse and carrot).
float PID_power[] = {0, 0}; // Motor power requested to acquire target speed.
float PID_error[] = {0, 0}; // Difference between actual and target.
float PID_lastError[] = {0, 0}; // For calculation of derivative.
float PID_integral[] = {0, 0}; // For accumulation of error.
float PID_derivative[] = {0, 0}; // For prediction of incoming error.
float PID_lastTarget[] = {0, 0}; // For signal generator calculation.
float PID_lastActual[] = {0, 0}; // For calculation of derivative.

const int PID_OVERLOAD_THRESHOLD = 450; // If the PID is behind this much, activates overload blink task.
short PID_blinkIdDisabled[] = {NULL, NULL, NULL}; // Placeholders for blink task ID's.
short PID_blinkIdOverloaded[] = {NULL, NULL, NULL};

// PID Loop constants for Proportional, Integral, and Derivative (Items in the acronym PID).
float PID_KU = 1;
float PID_KP = 0.06047619047619047619047619047619; //PID_POWER_MAX / PID_targetMax
float PID_KI = 0;
float PID_KD = 0;
/*
Ziegler-Nichols Method for Tuning PID Loops

	 1. Adjust Kp until oscillates evenly (oscillation does not gain or lose).
	 2. Ku = current Kp.
	 3. Time to complete one revolution = Tu.
	 4. Follow table:

	 Type |   Kp   |    Ki    |   Kd
	    P | 0.50Ku |        - |      -
	   PI | 0.45Ku | 1.2Kp/Tu |      -
	  PID | 0.60Ku |   2Kp/Tu | KpTu/8
*/

/*
Tuning checklist:
1. Start work on projectile calculations.
2. Application of Ziegler-Nichols method.
3. Find a reasonable integral limit.
4. Tweak signal generator.
5. Set system hysteresis.
6. Set reasonable motor overload threshold.
7. Find motor deadband.
*/

// TODO: Add info led for gun ready.

// Internal PID variables, not meant to be touched by outside modules. Also allows for debugging.
/*enum PID_Internal {filteredTarget, power, error, lastError, integral, derivative, lastTarget, lastActual};
float PID_internals[2][8];*/

// A single PID cycle.
void PID_cycle(PID_Side index) {
	if (PID_enabled) {
		// Determine how much time has elapsed (in minutes for RPM).
		PID_delta[index] = ((((float)time1[T1]) / 1000) / 60) - PID_deltaStart[index];
		if (PID_delta[index] <= 0) {
			writeDebugStreamLine("[PID] Running too fast (delta = 0s)! Skipping cycle.");
			return;
		}

		// Grab the actual sensor value, then scale it (if necessary).
		//writeDebugStreamLine("quad: %i, scale: %f, delta: %f, mirror: %d", SensorValue[PRT_gunLeftQuad], PID_INPUT_SCALE, PID_delta[index], PID_INPUT_MIRROR[index])
		PID_lastActual[index] = PID_actual[index];
		if (index == Left) {
			// Retrieve speed in RPM.
			PID_actual[index] = (SensorValue[PRT_gunLeftQuad] * PID_INPUT_SCALE) / PID_delta[index] * PID_INPUT_MIRROR[index];
			SensorValue[PRT_gunLeftQuad] = 0; // Clear encoder.
		} else if (index == Right) {
			PID_actual[index] = (SensorValue[PRT_gunRightQuad] * PID_INPUT_SCALE) / PID_delta[index] * PID_INPUT_MIRROR[index];
			SensorValue[PRT_gunRightQuad] = 0;
		}

		// Record the time between speed measurements.
		PID_deltaStart[index] = (((float)time1[T1]) / 1000) / 60;

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
		/*PID_filteredTarget[index] = PID_SIGNAL_GENERATOR_MAX * PID_lastTarget[index]
				+ (1.00 - PID_SIGNAL_GENERATOR_MAX) * PID_target[index];
		PID_lastTarget[index] = PID_filteredTarget[index];*/
		// Disabled for tuning purposes.
		PID_filteredTarget[index] = PID_target[index];
		PID_lastTarget[index] = PID_filteredTarget[index];

		// Calculate the error.
		PID_error[index] = PID_actual[index] - PID_filteredTarget[index];
		// Disabled for tuning purposes.
		/*if (abs(PID_error[index]) <= PID_SYSTEM_HYSTERESIS) {
			PID_error[index] = 0;
		}*/

		if (PID_INTEGRAL_ENABLED && PID_KI != 0) {
			// Accumulate error to fill the gap.
			if (abs(PID_error[index]) < PID_INTEGRAL_LIMIT) {
				PID_integral[index] += PID_error[index];
			}
		} else {
			PID_integral[index] = 0;
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
		/*if (PID_power[index] > 0 && PID_power[index] < PID_POWER_POSITIVE_DEADBAND
				&& PID_POWER_POSITIVE_DEADBAND != 0 && PID_power[index] != 0
				&& PID_actual[index] > PID_DEADBAND_THRESHOLD) {
			PID_power[index] = PID_POWER_POSITIVE_DEADBAND;
		} else if (PID_power[index] < 0 && PID_power[index] > PID_POWER_NEGATIVE_DEADBAND
				&& PID_POWER_NEGATIVE_DEADBAND != 0 && PID_power[index] != 0
				&& PID_actual[index] > (-1 * PID_DEADBAND_THRESHOLD)) {
			PID_power[index] = PID_POWER_NEGATIVE_DEADBAND;
		}*/

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
		if (abs(PID_error[index]) >= PID_OVERLOAD_THRESHOLD) {
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

// Input range in feet, output angular velocity in RPM.
float PID_angularSpeedAtRange(float range) {
	return 515.24 * sqrt(range);
}

task PID_autoTune() {
	PID_enabled = true;
	PID_target[0] = 500;
	wait1Msec(3000);
	const int samples = 20;
	const float magnitude = 0.01;
	float outputs[samples];
	float min;
	float mins[samples];
	float max;
	float maxs[samples];
	writeDebugStreamLine("Tuning with %i samples and magnitude of P change %f.", samples, magnitude);
	while (true) {
		min = PID_POWER_MAX;
		max = PID_POWER_MIN;
		for (int j = 0; j < samples; j++) {
			for (int i = 0; i < samples; i++) {
					outputs[i] = PID_power[0];
					min = MIN(min, outputs[i]);
					max = MAX(max, outputs[i]);
					wait1Msec(1000 / PID_INTERVALS_PER_SECOND);
			}
			writeDebugStreamLine("Min: %f, Max: %f for sample %i.", min, max, j);
			mins[j] = min;
			maxs[j] = max;
		}
		float lastMin = mins[0];
		float lastMax = maxs[0];
		int growing = 0;
		int shrinking = 0;
		int identical = 0;
		for (int i = 1; i < samples; i++) {
			if (mins[i] < lastMin)
				growing++;
			else if (mins[i] > lastMin)
				shrinking++;
			else
				identical++;

			if (maxs[i] > lastMax)
				growing++;
			else if (maxs[i] < lastMax)
				shrinking++;
			else
				identical++;

			lastMin = mins[i];
			lastMax = maxs[i];
		}
		if (growing > shrinking && growing > identical) {
			PID_KP -= magnitude;
			writeDebugStreamLine("Tune Score: G%i/S%i/I%i. Decreasing P to %f.", growing, shrinking, identical, PID_KP);
		}	else if (shrinking > growing && shrinking > identical) {
			PID_KP += magnitude;
			writeDebugStreamLine("Tune Score: G%i/S%i/I%i. Increasing P to %f.", growing, shrinking, identical, PID_KP);
		} else {
			writeDebugStreamLine("Tune Score: G%i/S%i/I%i. P = %f. TUNE COMPLETE.", growing, shrinking, identical, PID_KP);
			PID_target[0] = 0;
			return;
		}
	}
}

task PID_controller() {
	//bool batLowRetrieved = false;
	PID_deltaStart[Left] = ((float)time1[T1]) / 1000;
	PID_deltaStart[Right] = ((float)time1[T1]) / 1000;
	wait1Msec(1000 / PID_INTERVALS_PER_SECOND);
	while (true) {
		// Run each individual cycle per system.
		PID_cycle(Left);
		PID_cycle(Right);

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

		if (nTimeXX == 30) {
			startTask(PID_autoTune);
		}

		// Repeat at defined delta time, in Hertz.
		wait1Msec(1000 / PID_INTERVALS_PER_SECOND);
	}
}
