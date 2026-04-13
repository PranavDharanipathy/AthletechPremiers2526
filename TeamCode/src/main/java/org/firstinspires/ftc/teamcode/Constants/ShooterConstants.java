package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConstants {

    /// Y-Point that differentiates the turret pointing at the goal far/close position.
    public static double FAR_ZONE_CLOSE_ZONE_BARRIER = -35;

    public static double FAR_SIDE_FLYWHEEL_SHOOT_VELOCITY = 1400;
    public static double CLOSE_SIDE_FLYWHEEL_SHOOT_VELOCITY = 1150;

    public static double MIN_TURRET_POSITION_IN_DEGREES = -90, MAX_TURRET_POSITION_IN_DEGREES = 90;

    public static double HOOD_ANGLER_MIN_POSITION = 1;
    public static double HOOD_ANGLER_MAX_POSITION = 0;
    public static double HOOD_CLOSE_POSITION = 0.25;
    public static double HOOD_FAR_POSITION = 0.15;

    public static double TURRET_TICKS_PER_DEGREE = 91.0222222222;

    public static double TURRET_HOME_POSITION_INCREMENT = 200;

    public static double GOAL_X_POSITION_INCREMENT = 1;
    public static double GOAL_Y_POSITION_INCREMENT = 1;

    public static double TURRET_POSITIONAL_OFFSET = -0.7893685;

    /// The robot velocities must be greater than this for turret hysteresis control to be used.
    /// <p>
    /// Index 0 is translational, index 1 in angular (in radians).
    /// <p>
    /// Translational is in inches per second and angular is in radians per second.
    public static double[] TURRET_HYSTERESIS_CONTROL_ENGAGE_VELOCITY = {20, Math.toRadians(7)};

}