package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;

import java.util.ArrayList;
import java.util.List;

@Config
public class ShooterConstants {

    /// Y-Point that differentiates the turret pointing at the goal far/close position.
    public static double FAR_ZONE_CLOSE_ZONE_BARRIER = -35;

    public static double FAR_SIDE_FLYWHEEL_SHOOT_VELOCITY = 1800;
    public static double CLOSE_SIDE_FLYWHEEL_SHOOT_VELOCITY = 400;

    public static double MIN_TURRET_POSITION_IN_DEGREES = -90, MAX_TURRET_POSITION_IN_DEGREES = 90;

    public static double HOOD_ANGLER_MIN_POSITION = 0.7;
    public static double HOOD_ANGLER_MAX_POSITION = 1;

    //arbitrary positions
    public static double HOOD_CLOSE_POSITION = 0.25;
    public static double HOOD_FAR_POSITION = 0.15;

    public static double TURRET_TICKS_PER_DEGREE = 65.738;

    public static double TURRET_HOME_POSITION_INCREMENT = 200;

    public static double GOAL_X_POSITION_INCREMENT = 1;
    public static double GOAL_Y_POSITION_INCREMENT = 1;

    public static double TURRET_POSITIONAL_OFFSET = -1.775591;

    /// The robot velocities must be greater than this for turret hysteresis control to be used.
    /// <p>
    /// Index 0 is translational, index 1 in angular (in radians).
    /// <p>
    /// Translational is in inches per second and angular is in radians per second.
    public static double[] THC_ENGAGE_VELOCITY = {4, Math.toRadians(5)};

    /// Scalar value representing how influential acceleration is.
    /// <p>
    /// Should ONLY be tuned after the rest of THC.
    public static double THC_ACCELERATION_INFLUENCE = 0;

    /// Minimum acceleration required for acceleration to be used in pose prediction for THC.
    /// <p>
    /// Index 0 is translational, index 1 is heading.
    public static double[] THC_ACCELERATION_THRESHOLD = {0, 0};

    public static List<Double> CLOSE_HOOD_DISTANCES = new ArrayList<>(List.of(10.0, 40.0, 75.0));
    public static List<Double> CLOSE_HOOD_POSITIONS = new ArrayList<>(List.of(0.7, 1.0, 1.0));

    public static List<Double> FAR_HOOD_DISTANCES = new ArrayList<>(List.of(110.0, 150.0));
    public static List<Double> FAR_HOOD_POSITIONS = new ArrayList<>(List.of(1.0, 1.0));

    /// Minimum flywheel velocity-based hood correction for the correction to be used.
    public static double MINIMUM_FLYWHEEL_VELOCITY_HOOD_CORRECTION = 0.004;

    /// Minimum change in flywheel velocity-based hood correction for new hood correction to be used.
    public static double FLYWHEEL_VELOCITY_HOOD_CORRECTION_DEADBAND = 0.001;

    public static double HOOD_POSITION_FOR_MAX_SHOT_DISTANCE = 0.3;

    public static double FLYWHEEL_VELOCITY_HOOD_CORRECTION_INFLUENCE = 0;
    public static double FLYWHEEL_VELOCITY_HOOD_CORRECTION_MINIMUM = -1;
    public static double FLYWHEEL_VELOCITY_HOOD_CORRECTION_MAXIMUM = 1;

}