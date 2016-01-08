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
const int PID_POWER_MAX = 127; // Maximum and minimum amount of power for the PID to apply across all systems.
const int PID_POWER_MIN = -127;
const int PID_POWER_POSITIVE_DEADBAND = 32;
const int PID_POWER_NEGATIVE_DEADBAND = -32;
const int PID_INTEGRAL_LIMIT = 50; // Threshold for integral values.
const int PID_INTERVALS_PER_SECOND = 50; // Rate in hertz that the PID evaluates at.

const float PID_SENSOR_SCALE = 1; // If scaling of the encoder is necessary.
const float PID_INPUT_SCALE[] = {77.09, 77.09, 2.3}; // Set this to be {ENCODER TICKS / INCH, ENCODER TICKS / INCH, ENCODER TICKS / DEGREE}
float PID_target[] = {0, 0, 0}; // For setting the target height. Do not touch.
float PID_actual[] = {0, 0, 0}; // For detecting actual height. Do not touch.
const float PID_SIGNAL_GENERATOR_MAX[] = {0.96, 0.97, 0.98}; // Percentage value for "horse and carrot".
const int PID_SYSTEM_HYSTERESIS[] = {8, 6, 4}; // Solves for motor jogging without breaking PID.
float PID_targetMax[] = {2000, 750, 1023}; // Max height from the origin.

const int PID_OVERLOAD_THRESHOLD[] = {450, 200, 50}; // If the PID is behind this much, activates overload blink task.
short PID_blinkIdDisabled[] = {NULL, NULL, NULL}; // Placeholders for blink task ID's.
short PID_blinkIdOverloaded[] = {NULL, NULL, NULL};

// PID Loop constants for Proportional, Integral, and Derivative (Items in the acronym PID).
const float PID_KP[] = {0.80, 0.80, 0.80};
const float PID_KI[] = {0.00, 0.00, 0.00};
const float PID_KD[] = {0.00, 0.00, 0.00};

// How much controller button should add/subtract to the target.
const float PID_CONTROL_INCREMENTER_UP[] = {0.625, 0.1, 0.1};
const float PID_CONTROL_INCREMENTER_DOWN[] = {0.825, 0.1, 0.1};

// Internal PID variables, not meant to be touched by outside modules. Also allows for debugging.
enum PID_Internal {filteredTarget, power, error, lastError, integral, derivative, lastTarget, lastActual};
float PID_internals[3][8];

// A single PID cycle.
void PID_cycle(PID_Type index) {
	if (PID_enabled[index]) {
		// Grab the actual sensor value, then scale it (if necessary).
		PID_internals[index][lastActual] = PID_actual[index];
		if (index == MainLift) {
			PID_actual[index] = SensorValue[PRT_liftEncoder] * PID_SENSOR_SCALE[index];
		} /*else if (index == ClawLift) {
			PID_actual[index] = SensorValue[PRT_clawLiftEncoder] * PID_SENSOR_SCALE[index];
		} else if (index == ClawWrist) {
			PID_actual[index] = SensorValue[PRT_clawWristPot] * PID_SENSOR_SCALE[index];
		}*/

		// Target filter using a signal generator (horse and carrot method).
		// The horse and carrot method is a way of visualizing the movement of the PID's actuator
		// in relation to the target's movement. The "carrot" (target) is gradually dragged to another location
		// for the "horse" (actuator) to move to. The carrot does not teleport from one place to another.
		PID_internals[index][filteredTarget] = PID_SIGNAL_GENERATOR_MAX[index] * PID_internals[index][lastTarget]
				+ (1.00 - PID_SIGNAL_GENERATOR_MAX[index]) * PID_target[index];
		PID_internals[index][lastTarget] = PID_internals[index][filteredTarget];

		// Cap the target so it doesn't get too crazy.
		if (PID_target[index] > PID_targetMax[index]) {
			PID_target[index] = PID_targetMax[index];
		} else if (PID_target[index] < 0) {
			PID_target[index] = 0;
		}

		// Calculate the error.
		PID_internals[index][error] = PID_actual[index] - PID_internals[index][filteredTarget];
		if (abs(PID_internals[index][error]) <= PID_SYSTEM_HYSTERESIS[index]) {
			PID_internals[index][error] = 0;
		}

		// Automatic reset and on the spot calibration.
		if (index == MainLift) {
			// Button is not yet installed! Will cause problems.
			/*if (PID_actual[index] == 0 && (SensorValue[PRT_liftTouch] != 1)) {
				// If we are supposedly at 0, and are not touching the bottom,
				// increase the error to meet the bottom.
				PID_internals[index][error] = -100;
			} else if (PID_internals[index][error] < 0 && SensorValue[PRT_liftTouch] == 1) {
				// If we are at the bottom, and we are supposedly not,
				// reset the error, actual, and encoder.
				PID_internals[index][error] = 0;
				PID_actual[index] = 0;
				SensorValue[PRT_liftEncoder] = 0;
			}*/
		} else if (index == ClawLift) {
			// Not sure how we will do this yet.
		} else if (index == ClawWrist) {
			// Its a potentiometer, should (hopefully) handle itself.
		}

		if (PID_INTEGRAL_ENABLED[index]) {
			// Accumulate error to fill the gap.
			if (abs(PID_internals[index][error]) < PID_INTEGRAL_LIMIT[index]) {
				PID_internals[index][integral] += PID_internals[index][error];
			}
			if(PID_KI[index] == 0)
			    PID_internals[index][integral] = 0;
		} else {
			PID_internals[index][integral] = 0;
		}

		if (PID_DERIVATIVE_ENABLED[index]) {
			// Predict future change via derivative.
			PID_internals[index][derivative] = (PID_internals[index][error] - PID_internals[index][lastError]);
			PID_internals[index][lastError] = PID_internals[index][error];
		} else {
			PID_internals[index][derivative] = 0;
		}

		// Calculate the power necessary to correct the found error.
		// Negations are to keep error and power values in a logical context.
		PID_internals[index][power] = -1 * ((PID_KP[index] * PID_internals[index][error])
				+ (PID_KI[index] * PID_internals[index][integral])
				+ (PID_KD[index] * PID_internals[index][derivative]));

		// Limit the power sent to motors. (Can't exceed the byte)
		if (PID_internals[index][power] > PID_POWER_MAX) {
			PID_internals[index][power] = PID_POWER_MAX;
		} else if (PID_internals[index][power] < PID_POWER_MIN) {
			PID_internals[index][power] = PID_POWER_MIN;
		}

		// If the power level is below the deadband, increase it so that the
		// motor is actually moving toward its goal.
		if (PID_internals[index][power] > 0 && PID_internals[index][power] < PID_POWER_POSITIVE_DEADBAND[index]
				&& PID_POWER_POSITIVE_DEADBAND[index] != 0 && PID_internals[index][power] != 0) {
			PID_internals[index][power] = PID_POWER_POSITIVE_DEADBAND[index];
		} else if (PID_internals[index][power] < 0 && PID_internals[index][power] > PID_POWER_NEGATIVE_DEADBAND[index]
				&& PID_POWER_NEGATIVE_DEADBAND[index] != 0 && PID_internals[index][power] != 0) {
			PID_internals[index][power] = PID_POWER_NEGATIVE_DEADBAND[index];
		}

		// Set the motor power value.
		// Type cast to keep from wrongly setting the motors.
		if (index == MainLift) {
			motor[PRT_leftLiftMotors] = (int) PID_internals[index][power];
			motor[PRT_rightLiftMotors] = (int) PID_internals[index][power];
		} /*else if (index == ClawLift) {
			motor[PRT_clawLiftMotor] = (int) PID_internals[index][power];
		} else if (index == ClawWrist) {
			motor[PRT_clawWristMotor] = (int) PID_internals[index][power];
		}*/

		// Indicate that PID is struggling to meet its target.
		if (abs(PID_internals[index][error]) >= PID_OVERLOAD_THRESHOLD[index]
				&& (PID_internals[index][power] >= PID_POWER_POSITIVE_DEADBAND[index]
				|| PID_internals[index][power] <= PID_POWER_NEGATIVE_DEADBAND[index])) {
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
		/*if (PID_blinkIdDisabled[index] == NULL) {
			if (index == MainLift) {
				PID_blinkIdDisabled[index] = LED_startBlinkTask(Warning, Slow);
			} else if (index == ClawLift) {
				PID_blinkIdDisabled[index] = LED_startBlinkTask(Warning, Medium);
			} else if (index == ClawWrist) {
				PID_blinkIdDisabled[index] = LED_startBlinkTask(Warning, Fast);
			}
			if (PID_blinkIdOverloaded[index] != NULL) {
				LED_stopBlinkTask(PID_blinkIdOverloaded[index]);
				PID_blinkIdOverloaded[index] = NULL;
			}
		}*/
		// Be sure to stop the motor as well. (Externally once, rather than many times)
	}
}

task PID_controller() {
	bool batLowRetrieved = false;
	while (true) {
		// Run each individual cycle per system.
		PID_cycle(MainLift);
		PID_cycle(ClawLift);
		PID_cycle(ClawWrist);

		// Check the battery to initiate emergency PID shut off.
		if (batLowRetrieved == false && BAT_low == true) {
			batLowRetrieved = true;
			writeDebugStreamLine("[PID]: Activating emergency PID shut off to conserve power...");
			// The user can re-enable these with PID override.
			PID_enabled[MainLift] = false;
			PID_enabled[ClawLift] = false;
			PID_enabled[ClawWrist] = false;
		} else if (BAT_low == false) {
			batLowRetrieved = false;
		}

		// Repeat at defined delta time, in Hertz.
		wait1Msec(1000 / PID_INTERVALS_PER_SECOND);
	}
}

task PID_calibrateMainLift() {/*
	// Find the bottom of the lift.
	motor[PRT_leftLiftMotors] = -64;
	motor[PRT_rightLiftMotors] = -64;
	while (SensorValue[PRT_liftTouch] != 1) {
		// Wait until we hit the bottom...
		wait1Msec(10);
	}
	// Set the encoder to 0 for the PID loop.
	SensorValue[PRT_liftEncoder] = 0;

	// Find the max range of the lift. Usually somewhere around 2000.
	motor[PRT_leftLiftMotors] = 96;
	motor[PRT_rightLiftMotors] = 96;
	int lastEncoder = -10;
	while (SensorValue[PRT_liftEncoder] > lastEncoder) {
		// Wait until we stop increasing, meaning we've hit the top limit.
		lastEncoder = SensorValue[PRT_liftEncoder];
		wait1Msec(200);
	}
	motor[PRT_leftLiftMotors] = 0;
	motor[PRT_rightLiftMotors] = 0;
	PID_targetMax[MainLift] = SensorValue[PRT_liftEncoder];

	// Send to bottom.
	PID_target[MainLift] = 0;
*/}

task PID_calibrateClawLift() {/*
	// Find the bottom of the lift.
	motor[PRT_clawLiftMotor] = -64;
	int lastEncoder = 99999;
	while (SensorValue[PRT_clawLiftEncoder] < lastEncoder) {
		// Wait until we've stopped decreasing, meaning we've hit the bottom limit.
		lastEncoder = SensorValue[PRT_clawLiftEncoder];
		wait1Msec(200);
	}
	// Set the encoder to 0 for the PID loop.
	SensorValue[PRT_clawLiftEncoder] = 0;

	// Find the max range of the lift.
	motor[PRT_clawLiftMotor] = 96;
	lastEncoder = -10;
	while (SensorValue[PRT_clawLiftEncoder] > lastEncoder) {
		// Wait until we've stopped increasing, meaning we've hit the top limit.
		lastEncoder = SensorValue[PRT_clawLiftEncoder];
		wait1Msec(200);
	}
	motor[PRT_clawLiftMotor] = 0;
	PID_targetMax[ClawLift] = SensorValue[PRT_clawLiftEncoder];

	// Send to bottom.
	PID_target[ClawLift] = 0;
*/}

task PID_calibrateClawWrist() {/*
	// Reset at the bottom, while simultaneously checking to see if the potentiometer is installed correctly.
	int lastPot = 99999;
	while (SensorValue[PRT_clawWristPot] != 0) {
		lastPot = SensorValue[PRT_clawWristPot];
		//motor[PRT_clawWristMotor] = -64;
		wait1Msec(100);
		if (lastPot < SensorValue[PRT_clawWristPot]) {
			// Installed incorrectly...
			// Shut off the PID loop for this system to preserve the potentiometer and to keep from failure.
			motor[PRT_clawWristMotor] = 0;
			writeDebugStreamLine("[PID]: The claw wrist potentiometer is installed incorrectly. Reverse the motor or reposition the potentiometer.");
			PID_enabled[ClawWrist] = false;
			LED_startBlinkTask(Severe, Slow);
			break;
		}
	}
	motor[PRT_clawWristMotor] = 0;

	// Send to middle.
	PID_target[ClawWrist] = PID_targetMax[ClawWrist] / 2;
*/}

void PID_calibrate() {/*
	// Reset memory.
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 8; j++) {
			PID_internals[i][j] = 0;
		}
	}

	motor[PRT_clawMotor] = 127; // Close the claw.
	// Calibrate each individual system simultaneously.
	if (PID_enabled[MainLift] == true) {
		startTask(PID_calibrateMainLift);
	}
	if (PID_enabled[ClawLift] == true) {
		startTask(PID_calibrateClawLift);
	}
	if (PID_enabled[ClawWrist] == true) {
		startTask(PID_calibrateClawWrist);
	}

	//while (!(motor[PRT_liftMotor1] == 0 && motor[PRT_clawLiftMotor] == 0 && motor[PRT_clawWristMotor] == 0)) {
		// Wait until each system is calibrated.
		wait1Msec(100);
	}
	// Succesfully calibrated.
	motor[PRT_clawMotor] = -127; // Open the claw.

	// Enable PID loop.
	startTask(PID_controller);
	PID_target[MainLift] = 0;
*/}
