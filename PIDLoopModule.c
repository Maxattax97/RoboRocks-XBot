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

bool PID_enabled = true // Whether or not the PID is enabled for the system.
const bool PID_INTEGRAL_ENABLED = false;
const bool PID_DERIVATIVE_ENABLED = false;
const int PID_POWER_MAX = 127; // Maximum and minimum amount of power for the PID to apply.
const int PID_POWER_MIN = -127;
const int PID_DEADBAND_THRESHOLD = 210; // When speed is less than this number, apply deadband.
const int PID_POWER_POSITIVE_DEADBAND = 32; // Minimum power to set if not moving.
const int PID_POWER_NEGATIVE_DEADBAND = -32; // Minimum power to set if not moving
const int PID_INTEGRAL_LIMIT = 50; // Threshold for integral values.
const int PID_INTERVALS_PER_SECOND = 50; // Rate in hertz that the PID evaluates at.

const float PID_SENSOR_SCALE = 1; // If scaling of the encoder is necessary.
const float PID_INPUT_SCALE = 77.09; // Set this to be ENCODER TICKS / RPM.
float PID_target[] = {0, 0}; // For setting the target speed. Do not touch.
float PID_actual[] = {0, 0}; // For detecting actual speed. Do not touch.
const float PID_SIGNAL_GENERATOR_MAX = 0.96; // Percentage value for "horse and carrot".
const int PID_SYSTEM_HYSTERESIS = 8; // Solves for motor jogging without breaking PID.
float PID_targetMax = 2100; // Maximum speed.

const int PID_OVERLOAD_THRESHOLD = 450; // If the PID is behind this much, activates overload blink task.
short PID_blinkIdDisabled[] = {NULL, NULL, NULL}; // Placeholders for blink task ID's.
short PID_blinkIdOverloaded[] = {NULL, NULL, NULL};

// PID Loop constants for Proportional, Integral, and Derivative (Items in the acronym PID).
const float PID_KP = 0.8;
const float PID_KI = 0;
const float PID_KD = 0;

// How much controller button should add/subtract to the target.
const float PID_CONTROL_INCREMENTER_UP = 0.625;
const float PID_CONTROL_INCREMENTER_DOWN = 0.825;

// Internal PID variables, not meant to be touched by outside modules. Also allows for debugging.
enum PID_Internal {filteredTarget, power, error, lastError, integral, derivative, lastTarget, lastActual};
float PID_internals[2][8];

// A single PID cycle.
void PID_cycle(PID_Side index) {
	if (PID_enabled) {
		// Grab the actual sensor value, then scale it (if necessary).
		PID_internals[index][lastActual] = PID_actual[index];
		if (index == Left) {
			PID_actual[index] = SensorValue[PRT_gunQuadLeft] * PID_SENSOR_SCALE;
		} else if (index == Right) {
			PID_actual[index] = SensorValue[PRT_gunQuadRight] * PID_SENSOR_SCALE;
		}

		// Target filter using a signal generator (horse and carrot method).
		// The horse and carrot method is a way of visualizing the movement of the PID's actuator
		// in relation to the target's movement. The "carrot" (target) is gradually dragged to another location
		// for the "horse" (actuator) to move to. The carrot does not teleport from one place to another.
		PID_internals[index][filteredTarget] = PID_SIGNAL_GENERATOR_MAX * PID_internals[index][lastTarget]
				+ (1.00 - PID_SIGNAL_GENERATOR_MAX) * PID_target[index];
		PID_internals[index][lastTarget] = PID_internals[index][filteredTarget];

		// Cap the target so it doesn't get too crazy.
		if (PID_target[index] > PID_targetMax) {
			PID_target[index] = PID_targetMax;
		} else if (PID_target[index] < 0) {
			PID_target[index] = 0;
		}

		// Calculate the error.
		PID_internals[index][error] = PID_actual[index] - PID_internals[index][filteredTarget];
		if (abs(PID_internals[index][error]) <= PID_SYSTEM_HYSTERESIS) {
			PID_internals[index][error] = 0;
		}

		if (PID_INTEGRAL_ENABLED) {
			// Accumulate error to fill the gap.
			if (abs(PID_internals[index][error]) < PID_INTEGRAL_LIMIT) {
				PID_internals[index][integral] += PID_internals[index][error];
			}
			if(PID_KI == 0)
			    PID_internals[index][integral] = 0;
		} else {
			PID_internals[index][integral] = 0;
		}

		if (PID_DERIVATIVE_ENABLED) {
			// Predict future change via derivative.
			PID_internals[index][derivative] = (PID_internals[index][error] - PID_internals[index][lastError]);
			PID_internals[index][lastError] = PID_internals[index][error];
		} else {
			PID_internals[index][derivative] = 0;
		}

		// Calculate the power necessary to correct the found error.
		// Negations are to keep error and power values in a logical context.
		PID_internals[index][power] = -1 * ((PID_KP * PID_internals[index][error])
				+ (PID_KI * PID_internals[index][integral])
				+ (PID_KD * PID_internals[index][derivative]));

		// Limit the power sent to motors. (Can't exceed the byte)
		if (PID_internals[index][power] > PID_POWER_MAX) {
			PID_internals[index][power] = PID_POWER_MAX;
		} else if (PID_internals[index][power] < PID_POWER_MIN) {
			PID_internals[index][power] = PID_POWER_MIN;
		}

		// If the power level is below the deadband, increase it so that the
		// motor is actually moving toward its goal. Only apply deadband if below the
		// deadband threshold.
		if (PID_internals[index][power] > 0 && PID_internals[index][power] < PID_POWER_POSITIVE_DEADBAND
				&& PID_POWER_POSITIVE_DEADBAND != 0 && PID_internals[index][power] != 0
				&& PID_actual[index] > PID_DEADBAND_THRESHOLD) {
			PID_internals[index][power] = PID_POWER_POSITIVE_DEADBAND;
		} else if (PID_internals[index][power] < 0 && PID_internals[index][power] > PID_POWER_NEGATIVE_DEADBAND
				&& PID_POWER_NEGATIVE_DEADBAND != 0 && PID_internals[index][power] != 0
				&& PID_actual[index] > (-1 * PID_DEADBAND_THRESHOLD)) {
			PID_internals[index][power] = PID_POWER_NEGATIVE_DEADBAND;
		}

		// Set the motor power value.
		// Type cast to keep from wrongly setting the motors.
		if (index == Left) {
			motor[PRT_gunLeft1] = (int) PID_internals[index][power];
			motor[PRT_gunLeft2] = (int) PID_internals[index][power];
		} else if (index == Right) {
			motor[PRT_gunRight1] = (int) PID_internals[index][power];
			motor[PRT_gunRight2] = (int) PID_internals[index][power];
		}

		// Indicate that PID is struggling to meet its target.
		if (abs(PID_internals[index][error]) >= PID_OVERLOAD_THRESHOLD
				&& (PID_internals[index][power] >= PID_POWER_POSITIVE_DEADBAND
				|| PID_internals[index][power] <= PID_POWER_NEGATIVE_DEADBAND)) {
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
		PID_internals[index][error] = 0;
		PID_internals[index][lastError] = 0;
		PID_internals[index][integral] = 0;
		PID_internals[index][derivative] = 0;
		PID_internals[index][power] = 0;
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

task PID_controller() {
	//bool batLowRetrieved = false;
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

		// Repeat at defined delta time, in Hertz.
		wait1Msec(1000 / PID_INTERVALS_PER_SECOND);
	}
}
