//////////////////////////////////////////////////////////////////////////////////////
//                                    Gun Module                                    //
//////////////////////////////////////////////////////////////////////////////////////
// This module monitors and controls the gun system intelligently to maintain the   //
// motors, conserve battery life, designate when warm, and other functions.         //
//////////////////////////////////////////////////////////////////////////////////////

bool GUN_enabled = false;
//const float GUN_QUAD_TICKS_PER_REVOLUTION = 360.0;
//const float GUN_MAX_MOTOR_SPEED = 1.6667; // Speed of a 393 motor in revolutions per second.
//const float GUN_GEAR_RATIO = 3.0; // Gear ratio from QUAD ENCODER to gun wheel. (This was 21.0)
const int GUN_WARM_INCREMENT = 10; // How much motor power to increase per wait cycle.
const int GUN_LARGE_INCREMENT = 5; // How much to increase or decrease max motor power on a large scale.
const int GUN_SMALL_INCREMENT = 1; // How much to increase or decrease max motor power on a small scale.
const int GUN_DEFAULT_POWER = 75; // Default value for motor power. Resets to this when gun warm pressed.
//const float GUN_SPOOL_LOWER_POWER = 100.0; // Lower power level for spooling.
//const float GUN_SPOOL_LOWER_SPEED = (GUN_SPOOL_LOWER_POWER / 127.0) * GUN_MAX_MOTOR_SPEED * GUN_GEAR_RATIO; // The speed to drop to, then warm back up from.
const int GUN_CYCLES_PER_SECOND = 25; // In Hertz, how many times a second the cycle repeats. Used for measurements.
int GUN_maxMotorPower = GUN_DEFAULT_POWER; // Motor power when gun is fully warmed.
int GUN_power = 0; // How much power is being applied to gun motors currently.
float GUN_leftSpeed = 0; // Speed of the guns in rps. Maximum is theoretically 35 rps.
float GUN_rightSpeed = 0;
bool GUN_warming = false;
bool GUN_spool = false;
//bool GUN_spoolingDown = false;
float GUN_targetSpeed = 0;
float GUN_delta = 0;
float GUN_deltaStart = 0;
short GUN_blinkId = -1;
float GUN_HW_timeSinceLastBoost = 0;
float GUN_HW_boostTime = 0.65;
//float GUN_HW_spoolDownTime = 0.75;

// Static, timer based gun control system.
void GUN_hardWaitCycle() {
	if (GUN_enabled == true) {
		GUN_deltaStart = ((float)time1[T1]) / 1000;

		// If warming/spooling, check to see if a blink task exists yet. If not make one.
		if (GUN_blinkId == -1 && (GUN_spool == true || GUN_warming == true)) {
			GUN_blinkId = LED_startBlinkTask(Info, Fast);
		} else if (GUN_blinkId != -1 && GUN_spool == false && GUN_warming == false) {
			// If not warming, shut off blink task.
			LED_stopBlinkTask(GUN_blinkId);
			GUN_blinkId = -1;
		}

		/*if (GUN_spool == true || (GUN_warming == true && BAT_low == true)) {
			if (GUN_spoolingDown == false && (GUN_power <= 0 || GUN_HW_timeSinceLastBoost <= 0)) {
				if (GUN_power < 120) {
					GUN_power += GUN_WARM_INCREMENT;
					GUN_HW_timeSinceLastBoost = GUN_HW_boostTime;
				} else {
					if (GUN_power == 127) {
						GUN_power = 0;
						GUN_spoolingDown = true;
						GUN_HW_timeSinceLastBoost = GUN_HW_spoolDownTime;
					} else {
						// Indicate to the user via light that the guns are warmed up and are ready for firing.
						GUN_power = 127;
						GUN_HW_timeSinceLastBoost = GUN_HW_boostTime;
						if (GUN_blinkId != -1)
							LED_editBlinkTask(GUN_blinkId, Info, Solid);
					}
				}
			} else if (GUN_spoolingDown == true && GUN_HW_timeSinceLastBoost <= 0) {
				// Spool down until hitting a lower limit, then warm back up to full speed.
				GUN_spoolingDown = false;
				GUN_power = GUN_SPOOL_LOWER_POWER + GUN_WARM_INCREMENT;
				GUN_HW_timeSinceLastBoost = GUN_HW_boostTime;
				// Maintain solid light so user knows its still safe to fire.
				if (GUN_blinkId != -1)
					LED_editBlinkTask(GUN_blinkId, Info, Solid);
			}
		} else */if (GUN_warming == true) {
			if (GUN_power <= 0 || GUN_HW_timeSinceLastBoost <= 0) {
				if (GUN_power < GUN_maxMotorPower) { // Max power changed
					GUN_power += GUN_WARM_INCREMENT;
					GUN_HW_timeSinceLastBoost = GUN_HW_boostTime;
				} else {
					GUN_power = GUN_maxMotorPower; // Max power changed
					GUN_HW_timeSinceLastBoost = 0;
					if (GUN_blinkId != -1)
						LED_editBlinkTask(GUN_blinkId, Info, Solid);
				}
			}
		} else if (GUN_spool == false && GUN_warming == false) {
			// Cool guns by ramping motors down.
			//GUN_power = 0;
			//GUN_HW_timeSinceLastBoost = 0;
			GUN_maxMotorPower = GUN_DEFAULT_POWER;
			if (GUN_power > 0 && GUN_HW_timeSinceLastBoost <= 0) {
					GUN_power -= GUN_WARM_INCREMENT;
					GUN_HW_timeSinceLastBoost = GUN_HW_boostTime;
			} else if (GUN_HW_timeSinceLastBoost <= 0) {
					GUN_power = 0;
					GUN_HW_timeSinceLastBoost = 0;
			}
		}

		// Apply motor power.
		motor[PRT_gunLeft1] = GUN_power;
		motor[PRT_gunLeft2] = GUN_power;
		motor[PRT_gunLeft3] = GUN_power;
		motor[PRT_gunRight1] = GUN_power;
		motor[PRT_gunRight2] = GUN_power;
		motor[PRT_gunRight3] = GUN_power;

		wait1Msec(1000 / GUN_CYCLES_PER_SECOND);

		// Calculate speed over time interval.
		GUN_delta = (((float)time1[T1]) / 1000) - GUN_deltaStart;
		//if (GUN_warming == true || GUN_spool == true) {
			GUN_HW_timeSinceLastBoost -= GUN_delta;
		//}
	}
}


//// Dynamic, sensor based gun control system.
//void GUN_sensorCycle() {
//		GUN_deltaStart = ((float)time1[T1]) / 1000;

//		// Determine the speed the wheel should be theoretically moving at.
//		GUN_targetSpeed = ((((float)GUN_power) / 127.0) * GUN_maxMotorPower) * GUN_GEAR_RATIO;

//		// If warming/spooling, check to see if a blink task exists yet. If not make one.
//		if (GUN_blinkId == -1 && (GUN_spool == true || GUN_warming == true)) {
//			GUN_blinkId = LED_startBlinkTask(Info, Fast);
//		} else if (GUN_blinkId != -1 && GUN_spool == false && GUN_warming == false) {
//			// If not warming, shut off blink task.
//			LED_stopBlinkTask(GUN_blinkId);
//			GUN_blinkId = -1;
//		}

//		/*if (GUN_spool == true || (GUN_warming == true && BAT_low == true)) {
//			// Warm up guns, unless spooling down.
//			if (GUN_leftSpeed >= GUN_targetSpeed && GUN_rightSpeed >= GUN_targetSpeed && GUN_spoolingDown == false) {
//				if (GUN_power < 120) {
//					GUN_power += GUN_WARM_INCREMENT;
//				} else {
//					if (GUN_power == 127) {
//						// After hitting full speed, spool down to conserve energy.
//						GUN_power = 0;
//						GUN_spoolingDown = true;
//					} else {
//						// Indicate to the user via light that the guns are warmed up and are ready for firing.
//						GUN_power = 127;
//						if (GUN_blinkId != -1)
//							LED_editBlinkTask(GUN_blinkId, Info, Solid);
//					}
//				}
//			} else if (GUN_spoolingDown == true && (GUN_leftSpeed <= GUN_SPOOL_LOWER_SPEED || GUN_rightSpeed <= GUN_SPOOL_LOWER_SPEED)) {
//				// Spool down until hitting a lower limit, then warm back up to full speed.
//				GUN_spoolingDown = false;
//				GUN_power = GUN_SPOOL_LOWER_POWER + GUN_WARM_INCREMENT;
//				// Maintain solid light so user knows its still safe to fire.
//				if (GUN_blinkId != -1)
//					LED_editBlinkTask(GUN_blinkId, Info, Solid);
//			}
//		} else*/ if (GUN_warming == true) {
//			// Check if speed is at or above target speed for given motor power level.
//			if (GUN_leftSpeed >= GUN_targetSpeed && GUN_rightSpeed >= GUN_targetSpeed) {
//				if (GUN_power < GUN_maxMotorPower) {
//					// Increment motor power to reach a higher speed.
//					GUN_power += GUN_WARM_INCREMENT;
//				} else {
//					// Set power to full and designate ready for firing.
//					GUN_power = GUN_maxMotorPower;
//					if (GUN_blinkId != -1)
//						LED_editBlinkTask(GUN_blinkId, Info, Solid);
//				}
//			}
//		} else if (GUN_spool == false && GUN_warming == false) {
//			// Cool guns by shutting motors off.
//			GUN_power = 0;
//		}

//		// Apply motor power.
//		motor[PRT_gunLeft1] = GUN_power;
//		motor[PRT_gunLeft2] = GUN_power;
//		motor[PRT_gunRight1] = GUN_power;
//		motor[PRT_gunRight2] = GUN_power;

//		wait1Msec(1000 / GUN_CYCLES_PER_SECOND);

//		// Calculate speed over time interval.
//		GUN_delta = (((float)time1[T1]) / 1000) - GUN_deltaStart;
//		// FIX THIS LINE
//		// GUN_leftSpeed = (((float)SensorValue[PRT_gunLeftQuad]) / GUN_QUAD_TICKS_PER_REVOLUTION) / GUN_delta; // Revolutions per second.
//		GUN_leftSpeed = (((float)SensorValue[PRT_gunRightQuad]) / GUN_QUAD_TICKS_PER_REVOLUTION) / GUN_delta; // Revolutions per second.
//		GUN_rightSpeed = (((float)SensorValue[PRT_gunRightQuad]) / GUN_QUAD_TICKS_PER_REVOLUTION) / GUN_delta;

//		// Reset the sensors for the next sampling.
//		SensorValue[PRT_gunLeftQuad] = 0;
//		SensorValue[PRT_gunRightQuad] = 0;
//}

task GUN_controller() {
	while (true) {
		GUN_hardWaitCycle();
		//GUN_sensorCycle();
	}
}
