package org.firstinspires.ftc.teamcode.Constants;

import org.firstinspires.ftc.teamcode.util.MathUtil;

public class Models {

    public static double getScaledFlywheelKv(double unscaledKv, double currentVoltage) {

        final double SCALE_WEIGHT = 0.5;

        double scaledKv = (12.7 / currentVoltage) * unscaledKv;

        return SCALE_WEIGHT * scaledKv + (1 - SCALE_WEIGHT) * unscaledKv;
    }

    public static double getCloseHoodPosition(double distanceToGoal) {

        final double PIECEWISE_SWITCH = 80.3;

        double hoodPosition = 0.15;
//                distanceToGoal <= PIECEWISE_SWITCH ?
//                -0.00000917712 * FastMath.pow(distanceToGoal, 3) + 0.00218461 * FastMath.pow(distanceToGoal, 2) - 0.175314 * distanceToGoal + 4.88663 : // below/equal PIECEWISE_SWITCH
//                -0.0000009500073 * FastMath.pow(distanceToGoal, 4) + 0.0003606123 * FastMath.pow(distanceToGoal, 3) - 0.05110807 * FastMath.pow(distanceToGoal, 2) + 3.210404 * distanceToGoal - 75.32115; // above PIECEWISE_SWITCH

        return hoodPosition;
    }

    public static double getScaledMT1BotPoseFilterAlpha(double translationalVelMag) {

        final double SCALE_WEIGHT = 0.4;

        final double NOMINAL_ALPHA = 0.8;

        final double MIN_ALPHA = 0.7;
        final double MAX_ALPHA = 1.0;

        double scaledAlpha = (translationalVelMag / 50) * NOMINAL_ALPHA;

        return MathUtil.clamp(SCALE_WEIGHT * scaledAlpha + (1.0 - SCALE_WEIGHT) * NOMINAL_ALPHA, MIN_ALPHA, MAX_ALPHA);
    }

    /// For turret hysteresis control - the amount of time in the future where the robot's pose
    /// will be predicted based on its current pose and velocity as well as the turret's acceleration.
    public static double getTHCPosePredictionTime(double turretCurrentPosition, double turretPositionError) {//THC is turret hysteresis control

        final double NOMINAL_DT = 0.06; //dt in seconds
        final double SHOOTING_TIME = 0.08; //time it takes for ball to leave the shooter once contacting the flywheel
        final double MAX_VELOCITY = 0.08; //maximum velocity in rad/s
        final double MAX_ACCEL = 0.08; //maximum velocity is rad/s^2

        final double POSITIONAL_DIFFERENCE_SCALING = 1;
        final double ADJUSTMENT_POTENTIAL_SCALING = 1;
        final double DIRECTION_FORCE_SCALING = 1;

        final double errorRad = Math.abs(Math.toRadians(turretPositionError / ShooterConstants.TURRET_TICKS_PER_DEGREE));

        return
                POSITIONAL_DIFFERENCE_SCALING * (errorRad / MAX_VELOCITY) +
                        ADJUSTMENT_POTENTIAL_SCALING * (MAX_VELOCITY / MAX_ACCEL) +
                        DIRECTION_FORCE_SCALING * ((turretCurrentPosition + Math.signum(turretPositionError)) / MAX_ACCEL) +
                        SHOOTING_TIME +
                        NOMINAL_DT;
    }
}