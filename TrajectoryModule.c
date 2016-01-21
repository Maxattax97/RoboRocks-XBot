//////////////////////////////////////////////////////////////////////////////////////
//                                Trajectory Module                                 //
//////////////////////////////////////////////////////////////////////////////////////
// This modules provides crucial calculations to determine what output is required  //
// to most effectively deliver the ball into the net.                               //
//////////////////////////////////////////////////////////////////////////////////////

const int TRJ_INVALID = -77; // Designates an trajectory that is impossible.

const float TRJ_GUN_OFFSET = 1.5; // Distance from front of gun to front of sonar in inches.
const float TRJ_NET_OFFSET = 21.696; // Distance from the front of the low goal to the middle of the net opening.
const float TRJ_GUN_ANGLE = 23; // Declination in degrees of gun.
const float TRJ_GRAVITY = 386.088; // Acceleration in inches/s^2 of gravity.
const float TRJ_GUN_HEIGHT = 13; // Height in inches of the gun from the ground.
const float TRJ_NET_HEIGHT = 36.16; // Lower height of net opening in inches.
const float TRJ_BALL_RADIUS = 2; // Radius of a game ball in inches.
const float TRJ_WHEEL_RADIUS = 2; // Radius of the flywheel in inches.
const float TRJ_IPS_TO_RPM = 4.774648294; // Constant to multiply by inches/s to acquire rev/min.
const float TRJ_FPS_TO_RPM = 57.29564553094; // Constant to multiply by feet/s to acquire rev/min.
const float TRJ_RANGE_MIN = 5 * 12; // Range that is too close for a successful shot.
const float TRJ_RANGE_MAX = 16 * 12; // Range that is too far for a successful shot.
const float TRJ_BALL_MASS = 0.115; // Mass in pounds of the ball.
const float TRJ_WHEEL_MASS = 0.375; // Mass in pounds of the wheel.
float TRJ_MOMENTUM_RETENTION = 0.87; // How much momentum remains in the wheels after firing.

const float TRJ_INCH_TO_METER = 0.0254; // Multiply by inches to get meters.
const float TRJ_LB_TO_KG = 0.453592; // Multiply by pounds to get kilograms.
const float TRJ_REVPM_TO_RADPS = 0.10471975511966; // Multiply by revs/min to get rad/s.

int TRJ_calculation = 1; // Select which calculation to use.

// Determine whether or not the range is acceptable.
bool TRJ_validRange(float range) {
	if (range != SNR_INVALID && range >= TRJ_RANGE_MIN && range <= TRJ_RANGE_MAX && range != TRJ_INVALID) {
		return true;
	}
	return false;
}

// Input speed in inches/s, output angular velocity in RPM.
float TRJ_ballSpeedToRPM(float speed) {
	float wheelMomentOfInertia = (0.75 * TRJ_WHEEL_MASS * pow(TRJ_WHEEL_RADIUS, 2)) * 2;
	return 2 * PI * TRJ_WHEEL_RADIUS * (
		(TRJ_BALL_MASS * speed) / (wheelMomentOfInertia - TRJ_MOMENTUM_RETENTION * wheelMomentOfInertia));
}

// Input range in inches, output angular velocity in RPM.
float TRJ_angularSpeedAtRange(float range) {
	if (TRJ_validRange(range)) {
		if (TRJ_calculation == 1) {
			// Max's Equation
			return TRJ_ballSpeedToRPM( ((1 / cosDegrees(TRJ_GUN_ANGLE)) *
				sqrt((0.5 * TRJ_GRAVITY * pow(range, 2))
				/ (range * tanDegrees(TRJ_GUN_ANGLE) - (TRJ_NET_HEIGHT - TRJ_GUN_HEIGHT)))) );
		} else if (TRJ_calculation == 2) {
			// Firing Tables (air resistance)
			return TRJ_ballSpeedToRPM( TRJ_getFiringTable(range / 12) * 12 );
		} else if (TRJ_calculation == 3) {
			// Ryan, Parker, Sammy Equation
			float vBall = sqrt((2 * range * TRJ_GRAVITY)/(sinDegrees(2 * TRJ_GUN_ANGLE)));
			float angVel = sqrt((TRJ_BALL_MASS * pow(vBall, 2))/(2 * TRJ_MOMENTUM_RETENTION * TRJ_WHEEL_MASS * pow(TRJ_WHEEL_RADIUS, 2)));
			return (angVel * 30) / PI;
		}
	}
	return TRJ_INVALID;
}
