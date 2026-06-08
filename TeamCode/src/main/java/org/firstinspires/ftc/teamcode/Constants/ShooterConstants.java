package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;

import java.util.ArrayList;
import java.util.List;

@Config
public class ShooterConstants {

    /// Y-Point that differentiates the turret pointing at the goal far/close position.
    public static double FAR_ZONE_CLOSE_ZONE_BARRIER = -35;

    public static double MIN_TURRET_POSITION_IN_DEGREES = -90, MAX_TURRET_POSITION_IN_DEGREES = 90;

    public static double HOOD_ANGLER_MIN_POSITION = 0.78;
    public static double HOOD_ANGLER_MAX_POSITION = 0;

    //arbitrary positions
    public static double HOOD_CLOSE_POSITION = 0.25;
    public static double HOOD_FAR_POSITION = 0.15;

    public static double TURRET_TICKS_PER_DEGREE = 65.738;

    public static double TURRET_HOME_POSITION_INCREMENT = 200;

    public static double GOAL_X_POSITION_INCREMENT = 1;
    public static double GOAL_Y_POSITION_INCREMENT = 1;

    public static double TURRET_POSITIONAL_OFFSET = -1.775591;

    public static List<Double> CLOSE_HOOD_DISTANCES = new ArrayList<>(List.of( 55.78720363064907, 62.17491389340207, 67.76505977778645, 75.67361717828246, 83.65334103765882, 86.16299409450033, 91.67694645257207, 92.78325058675621, 109.96918200405516));
    public static List<Double> CLOSE_FLYWHEEL_VELOCITIES = new ArrayList<>(List.of(  1420.0,            1440.0,            1450.0,           1500.0,             1590.0,            1630.0,            1660.0,           1760.0,             1860.0));
    public static List<Double> CLOSE_HOOD_POSITIONS = new ArrayList<>(List.of(        0.0,                0.0,              0.0,              0.0,                0.0,               0.0,               0.0,              0.0,                0.0));

    public static List<Double> FAR_HOOD_DISTANCES = new ArrayList<>(List.of(126.69848229536962, 153.2564445950438));
    public static List<Double> FAR_FLYWHEEL_VELOCITIES = new ArrayList<>(List.of(   2100d,            2350d));
    public static List<Double> FAR_HOOD_POSITIONS = new ArrayList<>(List.of(         0.0,              0.0));

    public static double FLYWHEEL_VELOCITY_ALLOWABLE_ERROR = 30;

    /// Minimum flywheel velocity-based hood correction for the correction to be used.
    public static double MINIMUM_FLYWHEEL_VELOCITY_HOOD_CORRECTION = 0.004;

    /// Minimum change in flywheel velocity-based hood correction for new hood correction to be used.
    public static double FLYWHEEL_VELOCITY_HOOD_CORRECTION_DEADBAND = 0.001;

    public static double HOOD_POSITION_FOR_MAX_SHOT_DISTANCE = 0;

    public static double FLYWHEEL_VELOCITY_HOOD_CORRECTION_INFLUENCE = 0;
    public static double FLYWHEEL_VELOCITY_HOOD_CORRECTION_MINIMUM = -0.1;
    public static double FLYWHEEL_VELOCITY_HOOD_CORRECTION_MAXIMUM = 0.1;

    public static double FLYWHEEL_CONSIDERATION_VELOCITY = 60;

    /// in seconds
    public static double FLYWHEEL_SPEED_ADJUSTMENT_T = 0.6;

    public static double FLYWHEEL_SHOOT_SLIP_FACTOR = 0.173;

}