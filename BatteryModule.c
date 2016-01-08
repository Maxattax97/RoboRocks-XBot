//////////////////////////////////////////////////////////////////////////////////////
//                                  Battery Module                                  //
//////////////////////////////////////////////////////////////////////////////////////
// This module quietly monitors the battery levels in the background, and when      //
// the voltage starts to get low, sets up a warning LED. It also flips a variable   //
// so that other modules can be alerted of the shortage of power, and accomodate    //
// to conserve more energy, at the cost of manuever quality.                        //
//////////////////////////////////////////////////////////////////////////////////////

const float BAT_WARN = 6.6; // When to flash LED and output voltage to debug stream.
const float BAT_INTERVALS_PER_SECOND = 10; // How often the monitor will check per second.
bool BAT_low = false; // To warn other modules of low battery. (Emergency consumption reducer.)

task BAT_monitor() {
	writeDebugStreamLine("[Battery]: The battery's voltage is currently reading %1.3f V.", ((float) nAvgBatteryLevel) * 0.001);
	while (true) {
		// Grabs an average sampling of the battery level then scales. (20 samples)
		float batAvg = ((float) nAvgBatteryLevel) * 0.001;
		// Interesting stats: http://www.vexrobotics.com/wiki/Quazar%27s_Battery_Load_Test
		// If the battery level is low, start a warning signal.
		if (batAvg < BAT_WARN) {
			short id = LED_startBlinkTask(Severe, Fast);
			BAT_low = true;
			writeDebugStreamLine("[Battery]: The battery is low. Its voltage is reading %1.3f V", batAvg);
			// Enter next low battery check loop.
			while (true) {
				float batAvg = ((float) nAvgBatteryLevel) * 0.001;
				if (batAvg > BAT_WARN) {
					// Voltage change? Exit back into main loop.
					LED_stopBlinkTask(id);
					BAT_low = false;
					break;
				}
				wait1Msec(1000 / (BAT_INTERVALS_PER_SECOND / 2)); // Run at reduced pace, helps conserve battery.
			}
		}
		wait1Msec(1000 / BAT_INTERVALS_PER_SECOND);
	}
}
