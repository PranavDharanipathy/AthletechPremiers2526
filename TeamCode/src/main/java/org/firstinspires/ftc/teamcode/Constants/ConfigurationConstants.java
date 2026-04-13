package org.firstinspires.ftc.teamcode.Constants;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Systems.FlywheelPIDVSCoefficients;
import org.firstinspires.ftc.teamcode.Systems.TurretBasePIDFSCoefficients;

import java.util.ArrayList;
import java.util.List;

public class ConfigurationConstants {

    public static DcMotorSimple.Direction INTAKE_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction TRANSFER_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;

    public static Servo.Direction BLOCKER_SERVO_DIRECTION = Servo.Direction.REVERSE;

    public static Servo.Direction LIFT_SERVO_DIRECTION = Servo.Direction.REVERSE;

    /// Index 0 is the left crservo.
    /// <p>
    /// Index 1 is the right crservo.
    public static DcMotorSimple.Direction[] TURRET_BASE_DIRECTIONS = {
            DcMotorSimple.Direction.FORWARD,
            DcMotorSimple.Direction.FORWARD
    };

    /// Index 0 is the left servo.
    /// <p>
    /// Index 1 is the right servo.
    public static Servo.Direction HOOD_ANGLER_SERVO_DIRECTION = Servo.Direction.REVERSE;


    /// Index 0 is the left motor.
    /// <p>
    /// Index 1 is the right motor.
    public static DcMotorSimple.Direction[] FLYWHEEL_MOTOR_DIRECTIONS = {
            DcMotorSimple.Direction.REVERSE,
            DcMotorSimple.Direction.FORWARD
    };

    public static double[] TRANSFER_PDF_COEFFICIENTS = {0.000075, 0.00000136, 0.000428};

    public static double FLYWHEEL_ASSEMBLY_TOTAL_WEIGHT = 1534;
    public static double FLYWHEEL_SHAFT_DIAMETER = 8;
    public static double FLYWHEEL_MOTOR_CORE_VOLTAGE = 12;
    public static double FLYWHEEL_MOTOR_RPM = 6000;

    public static FlywheelPIDVSCoefficients FLYWHEEL_PIDVS_COEFFICIENTS = new FlywheelPIDVSCoefficients(
            0.01,
            0.01,
            0.0000001,
            0.00015,
            0.0000,
            0.000407,
            0.000001,
            0.83,
            80,
            100,
            0.5,
            0.3,
            -1, 1,
            -0.3, 0.3,
            -0.3, 0.3
    );

    public static double[] FLYWHEEL_KALMAN_FILTER_PARAMETERS = {0, 0, 1, 1};

    public static double FLYWHEEL_VELOCITY_MARGIN_OF_ERROR = 10;
    public static double FLYWHEEL_STABILITY_MARGIN_OF_ERROR = 10;

    public static TurretBasePIDFSCoefficients TURRET_PIDFS_COEFFICIENTS = new TurretBasePIDFSCoefficients(
            0.00011,
            new double[] {2e-8, 2e-8},
            new double[] {0.0000008, 0.000001},
            0.0008,
            0.0,
            0.0266,
            new double[] {400, 400},
            new double[] {0.12, 0.145},
            new double[] {0, 0},
            new double[] {0.9, 0.9},
            1,
            new double[] {3, 1},
            0,
            -0.45,
            0.45
    ).withTuning(true);

    public static List<Double> TURRET_PD_POSITIONS = new ArrayList<>(List.of(-14000.0,   -13000.0,   -12000.0,   -11000.0,   -10000.0,   -9000.0,    -8000.0,    -7000.0,    -6000.0,    -5000.0,      -4000.0,    -3000.0,    -2000.0,     -1000.0,       0.0,        1000.0,    2000.0,     3000.0,    4000.0,     5000.0,     6000.0,    7000.0,     8000.0,     9000.0,   10000.0));
    public static List<Double> TURRET_KPS =          new ArrayList<>(List.of(0.000215,    0.0002,     0.0002,     0.00021,    0.00021,   0.00022,     0.0002,    0.00017,    0.00014,     0.0001,      0.000095,   0.00009,   0.0000885,   0.00008985,  0.0000995,   0.0000899, 0.000088,   0.00009,  0.000092,    0.0000833,   0.00008,   0.00008,    0.00008,    0.00008,   0.0001));
    public static List<Double> TURRET_KDS =          new ArrayList<>(List.of( 0.002,      0.0044,     0.0044,     0.0043,     0.0043,    0.0037,      0.0035,     0.003,      0.003,      0.0012,       0.001,      0.0035,    0.0035,       0.0035,      0.0035,      0.0035,   0.0035,     0.0035,     0.0035,     0.0035,     0.0039,    0.0039,      0.004,     0.004,    0.0015));
    public static List<Double> TURRET_FEEDFORWARD_POSITIONS = new ArrayList<>(List.of(-14000.0,   -13000.0,  -12000.0,      -11000.0,   -10000.0,   -9000.0,    -8000.0,    -7000.0,    -6000.0,    -5000.0,      -4000.0,   -3000.0,    -2000.0,    -1000.0,    0.0,     1000.0,      2000.0,     3000.0,     4000.0,    5000.0,    6000.0,      7000.0,     8000.0,     9000.0,   10000.0));
    public static List<Double> TURRET_KFS =                   new ArrayList<>(List.of(0.0000022,  0.0000022,  0.00000225,  0.00000225,  0.0000024, 0.00000285,  0.000003,  0.0000032,  0.0000031,   0.0000022,   0.0000022,  0.000002,   0.000001,  0.00000082,  0.0,   0.00000082,  0.00000082,  0.000001,   0.000001,  0.000001,  0.0000012,  0.00000135,  0.0000018,  0.000002,  0.000003));

    public static double TURRET_KF_RESISTANCE_ENGAGE_ERROR = 2000;

}