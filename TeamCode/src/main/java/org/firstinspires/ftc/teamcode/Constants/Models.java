package org.firstinspires.ftc.teamcode.Constants;

import org.firstinspires.ftc.teamcode.util.MathUtil;

public class Models {

    public static double getScaledMT1BotPoseFilterAlpha(double translationalVelMag) {

        final double SCALE_WEIGHT = 0.5;

        final double NOMINAL_ALPHA = 0.8;

        final double MIN_ALPHA = 0.67;
        final double MAX_ALPHA = 1.0;

        double scaledAlpha = (translationalVelMag / 48) * NOMINAL_ALPHA;

        return MathUtil.clamp(SCALE_WEIGHT * scaledAlpha + (1.0 - SCALE_WEIGHT) * NOMINAL_ALPHA, MIN_ALPHA, MAX_ALPHA);
    }

    public static double getBallSpeedFromFlywheel(double flywheelVelocity) {

        double flywheelRPS = (flywheelVelocity / 28d);

        return ShooterConstants.FLYWHEEL_SHOOT_SLIP_FACTOR * flywheelRPS * Math.PI * ConfigurationConstants.FLYWHEEL_DIAMETER;
    }
}