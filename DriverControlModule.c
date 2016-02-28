//////////////////////////////////////////////////////////////////////////////////////
//                               Driver Control Module                              //
//////////////////////////////////////////////////////////////////////////////////////
// This module allows a wide variety of configurations per driver. It allows drivers//
// to swap position at the change of the DRV_CURRENT_DRIVER variable to a defined   //
// enum value. New drivers can easily be added and have an effect throughout the    //
// rest of the programming. It also cleans controller values to be used later by    //
// other systems in the code, such as button down "events", or trimming joystick    //
// values.                                                                          //
//////////////////////////////////////////////////////////////////////////////////////

// Set the DRV_CURRENT_DRIVER variable to whoever is preconfigured.
// Alternatively, you can also add your own settings.
// Add a name to DRV_Driver enum, then add configurations in DRV_setupConfig() using an if statement.
enum DRV_Driver {Default, Parker, Zander, Ryan, Sammy};
// If you are unsure, leave the setting on Default. It is configured to be efficient and intuitive.
const DRV_Driver DRV_CURRENT_DRIVER = Sammy;
const int DRV_BUTTON_COUNT = 32;
short DRV_simulatedButtonPress = -1;

bool DRV_controllerOverridden = false;

// Enum values to bind program functions with buttons.
enum DRV_RemoteFunction {MecanumRightNormal = 0, MecanumRightStrafe, MecanumLeftNormal, MecanumLeftStrafe, MecanumRotate,
	OmniLeft, OmniRight, OmniForward, OmniRotate, OmniMirrorForward, OmniMirrorRotate, ToggleMirror,
	FeedLowerIn, FeedLowerOut, FeedUpperIn, FeedUpperOut, FeedUpperInSmall,
	FeedLowerInSecondary, FeedLowerOutSecondary, FeedUpperInSecondary, FeedUpperOutSecondary,
	GunWarm, GunSpool, GunIncrement, GunDecrement,
	GunSmallIncrement, GunSmallDecrement, GunResetTarget, SonarCapture, Ping, Override, UNASSIGNED = 99};

// These arrays are available for other parts of the code to retrieve cleaned values.
DRV_RemoteFunction DRV_config[DRV_BUTTON_COUNT]; // This array returns the bound button/joystick value from the controller.
bool DRV_controllerButtonsDown[DRV_BUTTON_COUNT]; // This array returns only BUTTONS that have been pushed down ONCE. Comparable to onDown() event.

int DRV_currentController = 3; // 1: Primary | 2: Secondary | 3: Both
short DRV_controllerWarningLed = -1;
const int DRV_JOYSTICK_THRESHOLD = 15; // The trim for the joystick values.
const int DRV_INTERVALS_PER_SECOND = 50; // Hertz rate to check buttons.

void DRV_setupConfig() {
	writeDebugStreamLine("[Config]: The designated driver index is %i.", DRV_CURRENT_DRIVER);
	//// DEFAULT ////
	// Setup default button binds, then let DRV_CURRENT_DRIVER override. No hollow values that way.
	// Joystick slots should only be channels or UNASSIGNED.
	// Wheel controls should never be buttons.
	DRV_config[MecanumRightNormal] = UNASSIGNED; // Joystick channel that controls right side wheels forward and backward movement.
	DRV_config[MecanumRightStrafe] = UNASSIGNED; // Joystick channel that controls the right side wheels strafing movement.
	DRV_config[MecanumLeftNormal] = UNASSIGNED; // Joystick channel that controls the left side wheels forward and backward movement.
	DRV_config[MecanumLeftStrafe] = UNASSIGNED; // Joystick channel that controls the left side wheels strafing movement.
	DRV_config[MecanumRotate] = UNASSIGNED; // Joystick channel that controls the rotating ability of both sides.
	DRV_config[OmniLeft] = Ch3Xmtr2; // Joystick channel that controls the left side of omni wheels.
	DRV_config[OmniRight] = Ch2Xmtr2; // Joystick channel that controls the right side of omni wheels.
	DRV_config[OmniForward] = UNASSIGNED; // Joystick channel that moves omni wheels forward and backward.
	DRV_config[OmniRotate] = UNASSIGNED; // Joystick channel that turns omni wheels left and right.
	DRV_config[OmniMirrorForward] = UNASSIGNED; // Joystick channel that moves omni wheels forward and backward in a mirrored fashion.
	DRV_config[OmniMirrorRotate] = UNASSIGNED; // Joystick channel that turns omni wheels left and right in a mirrored fashion.
	DRV_config[ToggleMirror] = UNASSIGNED; // Toggles the reversing/mirroring of wheel control.
	DRV_config[FeedLowerIn] = Btn5U; // Lower belt feed pulls items into robot.
	DRV_config[FeedLowerOut] = Btn5D; // Lower belt feed pushes items out of robot.
	DRV_config[FeedUpperIn] = Btn6U; // Upper belt feed pulls items into robot.
	DRV_config[FeedUpperOut] = Btn6D; // Upper belt feed pushes items out of robot.
	DRV_config[FeedLowerInSecondary] = Btn5UXmtr2; // Lower belt feed pulls items into robot.
	DRV_config[FeedLowerOutSecondary] = Btn5DXmtr2; // Lower belt feed pushes items out of robot.
	DRV_config[FeedUpperInSecondary] = Btn6UXmtr2; // Upper belt feed pulls items into robot.
	DRV_config[FeedUpperOutSecondary] = Btn6DXmtr2; // Upper belt feed pushes items out of robot.
	DRV_config[FeedUpperInSmall] = UNASSIGNED; // Was temporarily used for small feed adjustments.
	DRV_config[GunWarm] = Btn8D; // Toggles speeding up of both firing wheels.
	DRV_config[GunSpool] = UNASSIGNED; // Gun speeds up until warm, then repeats cooling and warming to maintain speed and battery.
	DRV_config[GunIncrement] = Btn7U; // Increments max gun speed.
	DRV_config[GunDecrement] = Btn7D; // Decrements max gun speed.
	DRV_config[GunSmallIncrement] = Btn7R; // Increments max gun speed by a small amount.
	DRV_config[GunSmallDecrement] = Btn7L; // Decrements max gun speed by a small amount.
	DRV_config[GunResetTarget] = Btn8U; // Sets the target back to default range.
	DRV_config[SonarCapture] = UNASSIGNED; // Saves current sonar reading to target range.
	DRV_config[Ping] = UNASSIGNED; // Flashes lights on cortex to indicate responsiveness.
	DRV_config[Override] = Btn8L; // Overrides the PID loop, and sets back the old, hard-wait gun system.

	if (DRV_CURRENT_DRIVER == Parker) {
		//// PARKER ////
		DRV_config[MecanumRightNormal] = Ch3;
		DRV_config[MecanumRightStrafe] = Ch4;
		DRV_config[MecanumLeftNormal] = Ch3;
		DRV_config[MecanumLeftStrafe] = Ch4;
		DRV_config[MecanumRotate] = Ch1;
	} else if (DRV_CURRENT_DRIVER == Zander) {
		//// ZANDER ////
		DRV_config[OmniRight] = UNASSIGNED;
		DRV_config[OmniLeft] = UNASSIGNED;
		DRV_config[OmniForward] = Ch3;
		DRV_config[OmniRotate] = Ch4;
		DRV_config[OmniMirrorForward] = Ch2;
		DRV_config[OmniMirrorRotate] = Ch1;
	} else if (DRV_CURRENT_DRIVER == Ryan) {
		//// RYAN ////
		DRV_config[MecanumRightNormal] = Ch3;
		DRV_config[MecanumRightStrafe] = Ch4;
		DRV_config[MecanumLeftNormal] = Ch3;
		DRV_config[MecanumLeftStrafe] = Ch4;
		DRV_config[MecanumRotate] = Ch1;
	} else if (DRV_CURRENT_DRIVER == Sammy) {
		//// SAMMY ////
		DRV_config[OmniRight] = Ch2Xmtr2;
		DRV_config[OmniLeft] = Ch3Xmtr2;
		DRV_config[OmniForward] = UNASSIGNED;
		DRV_config[OmniRotate] = UNASSIGNED;
		DRV_config[OmniMirrorForward] = UNASSIGNED;
		DRV_config[OmniMirrorRotate] = UNASSIGNED;
		DRV_config[ToggleMirror] = Btn8RXmtr2;
	}
}

// Trims the joystick channel's value. This eliminates joystick noise, and motor whining (Yay!).
int DRV_trimChannel(DRV_RemoteFunction channel, int trim = DRV_JOYSTICK_THRESHOLD) {
	int value = vexRT[DRV_config[channel]];
	if (abs(value) <= trim) {
		return 0;
	}
	return value;
}

// Accepts a button/joystick from one controller and translates it to the opposite controller.
// If reverse is false, will return controller 1's corresponding button. Otherwise, will return controller 2's.
short DRV_translateXmtr(short button, bool reverse = false) {
	if (reverse == false) {
		switch (button) {
			case Ch1Xmtr2   : return Ch1;
			case Ch2Xmtr2   : return Ch2;
			case Ch3Xmtr2   : return Ch3;
			case Ch4Xmtr2   : return Ch4;
			case Btn5UXmtr2 : return Btn5U;
			case Btn5DXmtr2 : return Btn5D;
			case Btn6UXmtr2 : return Btn6U;
			case Btn6DXmtr2 : return Btn6D;
			case Btn7UXmtr2 : return Btn7U;
			case Btn7DXmtr2 : return Btn7D;
			case Btn7LXmtr2 : return Btn7L;
			case Btn7RXmtr2 : return Btn7R;
			case Btn8UXmtr2 : return Btn8U;
			case Btn8DXmtr2 : return Btn8D;
			case Btn8LXmtr2 : return Btn8L;
			case Btn8RXmtr2 : return Btn8R;
			default: return button;
		}
	} else if (reverse == true) {
		switch (button) {
			case Ch1   : return Ch1Xmtr2;
			case Ch2   : return Ch2Xmtr2;
			case Ch3   : return Ch3Xmtr2;
			case Ch4   : return Ch4Xmtr2;
			case Btn5U : return Btn5UXmtr2;
			case Btn5D : return Btn5DXmtr2;
			case Btn6U : return Btn6UXmtr2;
			case Btn6D : return Btn6DXmtr2;
			case Btn7U : return Btn7UXmtr2;
			case Btn7D : return Btn7DXmtr2;
			case Btn7L : return Btn7LXmtr2;
			case Btn7R : return Btn7RXmtr2;
			case Btn8U : return Btn8UXmtr2;
			case Btn8D : return Btn8DXmtr2;
			case Btn8L : return Btn8LXmtr2;
			case Btn8R : return Btn8RXmtr2;
			default: return button;
		}
	}
	return button;
}

#if DEBUG == 1
// Simulates a button press for debugging or controls.
void DRV_simulateButtonPress(short button) {
	DRV_simulatedButtonPress = button;
}
#endif

// The loop that handles button down "events".
task DRV_buttonHandler()
{
	// Setup the drivers.
	DRV_setupConfig();
	// Clear the buttons.
	for (int i = 0; i < DRV_BUTTON_COUNT; i++) {
		DRV_controllerButtonsDown[i] = false;
	}
	bool lastController[DRV_BUTTON_COUNT] = DRV_controllerButtonsDown;
	// Set all button presses to true if the button is down.
	// After button value is checked, must be cancelled! Use DRV_controllerButtonsDown[button] = false
	while(true) {
		for (int button = 0; button < DRV_BUTTON_COUNT; button++) {
			if (vexRT[DRV_config[button]] == true || DRV_simulatedButtonPress == DRV_config[button]) {
				if (lastController[button] != true) {
					// New button down. Add it to the Controller.
					writeDebugStreamLine("[Control]: Button %i down.", button);
					DRV_controllerButtonsDown[button] = true;
					lastController[button] = true;
				}
			} else {
				lastController[button] = false;
			}
		}
		DRV_simulatedButtonPress = -1; // Reset simulated button to a non-existant one.
		// Wait at a set interval, to allow processing power to other modules.
		wait1Msec(1000 / DRV_INTERVALS_PER_SECOND);
	}
}
