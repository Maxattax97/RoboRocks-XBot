//////////////////////////////////////////////////////////////////////////////////////
//                                    Blink Module                                  //
//////////////////////////////////////////////////////////////////////////////////////
// This module allows other systems to indicate their statuses via the use of three //
// LED's installed on the Cortex. Rather than using seperate methods per blink, we  //
// create one that indicates each blink at regular intervals and rates. This        //
// simplifies the problem significantly, and lends more CPU power to other          //
// processes.                                                                       //
//////////////////////////////////////////////////////////////////////////////////////

// Enum values for different blink task settings.
enum LED_Level {Info = 1, Warning, Severe, Gun, UNASSIGNED = 99};
enum LED_Rate {Solid = 1, Slow, Medium, Fast, Irregular, UNASSIGNED = 99};

// LED_Task structure, comparable to OOP's Class.
typedef struct {
	LED_Level level;
	LED_Rate rate;
} LED_Task;

LED_Task LED_tasks[15];

task LED_blink() {
	// Setup the array.
	// (sizeof(LED_tasks) / 4) is a way to find array size. Each struct is worth 4 bytes.
	for (int i = 0; i < (sizeof(LED_tasks) / 4); i++) {
		LED_tasks[i].level = UNASSIGNED;
		LED_tasks[i].rate = UNASSIGNED;
	}

	// Reset LED's.
	SensorValue[PRT_ledG] = 0;
	SensorValue[PRT_ledY] = 0;
	SensorValue[PRT_ledR] = 0;

	// Loop through each setting and apply appropriately.
	// If there are overlapping blink tasks, more recent ones (likely more important), will be shown.
	while (true) {
		for (int cycle = 1; cycle < (6 + 1); cycle++) {
			for (int i = (sizeof(LED_tasks) / 4) - 1; i > -1; i--) {
				// Check to make sure it is not an empty task.
				if (LED_tasks[i].level != UNASSIGNED && LED_tasks[i].rate != UNASSIGNED) {
					short index;
					if (LED_tasks[i].level == Info) {
						index = PRT_ledG;
					} else if (LED_tasks[i].level == Warning) {
						index = PRT_ledY;
					} else if (LED_tasks[i].level == Severe) {
						index = PRT_ledR;
					} else if (LED_tasks[i].level == Gun) {
						index = PRT_ledGun;
					}

					// Blink at set rates. There are 6 intervals in a second. Each interval lights an LED based on rate.
					// Rate  |  1   2   3   4   5   6
					// -------------------------------
					// Solid : [X] [X] [X] [X] [X] [X]
					// Fast  : [X] [ ] [X] [ ] [X] [ ]
					// Medium: [X] [ ] [ ] [X] [ ] [ ]
					// Slow  : [X] [ ] [ ] [ ] [ ] [ ]
					// Irreg : [X] [ ] [X] [ ] [ ] [ ]
					if (cycle == 1) {
						SensorValue[index] = 1;
					} else if (cycle == 3 && (LED_tasks[i].rate == Fast || LED_tasks[i].rate == Irregular)) {
						SensorValue[index] = 1;
					} else if (cycle == 4 && LED_tasks[i].rate == Medium) {
						SensorValue[index] = 1;
					} else if (cycle == 5 && LED_tasks[i].rate == Fast) {
						SensorValue[index] = 1;
					} else if (LED_tasks[i].rate == Solid) {
						SensorValue[index] = 1;
					} else {
						SensorValue[index] = 0;
					}
				}
			}
			wait1Msec(1000 / 6);
		}
	}
}

// Function that allows other modules to set up a blink task.
short LED_startBlinkTask(LED_Level level, LED_Rate rate) {
	// Find an empty slot, then fill it with a new task.
	// Return identifier.
	for (int i = (sizeof(LED_tasks) / 4) - 1; i > -1; i--) {
		if (LED_tasks[i].level == UNASSIGNED && LED_tasks[i].rate == UNASSIGNED) {
			LED_tasks[i].level = level;
			LED_tasks[i].rate = rate;
			return i;
		}
	}
	// If there are too many LED_tasks, return null.
	return NULL;
}

// Allows other modules to shut off blink tasks with an identifier.
bool LED_stopBlinkTask(short identifier) {
	// Check to see if given identifier is real, then clear it.
	// Return with success value.
	if (LED_tasks[identifier].level != UNASSIGNED && LED_tasks[identifier].rate != UNASSIGNED) {
		switch (LED_tasks[identifier].level) {
			case Info:
				SensorValue[PRT_ledG] = 0;
				break;
			case Warning:
				SensorValue[PRT_ledY] = 0;
				break;
			case Severe:
				SensorValue[PRT_ledR] = 0;
				break;
		}
		LED_tasks[identifier].level = UNASSIGNED;
		LED_tasks[identifier].rate = UNASSIGNED;
		return true;
	} else {
		return false;
	}
}
/*
LED_Level LED_getBlinkTaskLevel(short identifier) {
	return LED_tasks[identifier].level;
}

LED_Rate LED_getBlinkTaskRate(short identifier) {
	return LED_tasks[identifier].rate;
}
*/
void LED_editBlinkTask(short identifier, LED_Level level, LED_Rate rate) {
	LED_tasks[identifier].level = level;
	LED_tasks[identifier].rate = rate;
}

/* Clear the blink task memory.
void LED_clearBlinkTasks() {
	for (int i = 0; i < (sizeof(LED_tasks) / 4); i++) {
		LED_tasks[i].level = UNASSIGNED;
		LED_tasks[i].rate = UNASSIGNED;
	}
	SensorValue[PRT_ledG] = 0;
	SensorValue[PRT_ledY] = 0;
	SensorValue[PRT_ledR] = 0;
}
*/
