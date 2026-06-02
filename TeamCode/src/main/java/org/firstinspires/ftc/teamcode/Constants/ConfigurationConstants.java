package org.firstinspires.ftc.teamcode.Constants;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Systems.FlywheelPIDVSCoefficients;
import org.firstinspires.ftc.teamcode.Systems.TurretBasePIDFCoefficients;

import java.util.ArrayList;
import java.util.List;

public class ConfigurationConstants {

    public static DcMotorSimple.Direction INTAKE_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction TRANSFER_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;

    public static Servo.Direction BLOCKER_SERVO_DIRECTION = Servo.Direction.REVERSE;

    public static Servo.Direction INTAKE_DROPDOWN_SERVO_DIRECTION = Servo.Direction.FORWARD;

    public static Servo.Direction LIFT_SERVO_DIRECTION = Servo.Direction.REVERSE;

    /// Index 0 is the left crservo.
    /// <p>
    /// Index 1 is the right crservo.
    public static DcMotorSimple.Direction[] TURRET_BASE_DIRECTIONS = {
            DcMotorSimple.Direction.REVERSE,
            DcMotorSimple.Direction.REVERSE
    };

    /// Index 0 is the left servo.
    /// <p>
    /// Index 1 is the right servo.
    public static Servo.Direction HOOD_ANGLER_SERVO_DIRECTION = Servo.Direction.REVERSE;


    /// Index 0 is the left motor.
    /// <p>
    /// Index 1 is the right motor.
    public static DcMotorSimple.Direction[] FLYWHEEL_MOTOR_DIRECTIONS = {
            DcMotorSimple.Direction.FORWARD,
            DcMotorSimple.Direction.REVERSE
    };

    public static double[] TRANSFER_PDF_COEFFICIENTS = {0.00012, 0.000005, 0.00042};

    public static double FLYWHEEL_ASSEMBLY_TOTAL_WEIGHT = 829;
    public static double FLYWHEEL_SHAFT_DIAMETER = 8;
    public static double FLYWHEEL_MOTOR_CORE_VOLTAGE = 12;
    public static double FLYWHEEL_MOTOR_RPM = 6000;

    public static FlywheelPIDVSCoefficients FLYWHEEL_PIDVS_COEFFICIENTS = new FlywheelPIDVSCoefficients(
            0.0008 /*0.00072*/,
            0.000043,
            0,
            0.0000275,
            0.0000075,
            0.0003422,
            0.000001,
            1.35,
            100,
            200,
            0.88,
            0.945,
            0.65,
            12.17,
            -1, 1,
            -0.3, 0.3,
            -0.1, 0.1
    );

    public static double[] FLYWHEEL_KALMAN_FILTER_PARAMETERS = {56, 40, 4.25, 5};

    public static double FLYWHEEL_VELOCITY_MARGIN_OF_ERROR = 10;
    public static double FLYWHEEL_STABILITY_MARGIN_OF_ERROR = 10;

    public static double[] TURRET_VELOCITY_COEFFICIENTS = {
            0.0000008,
            0.00000015,
            0.93,
            0.0000001,
            0,
            -0.25,
            0.25,
            0.0000000035,
            0.0785,
            0.0000011,
            0.0000017,
            12.35,
            0.6,
            0.05,
            0.0479477,
            0.00000883825,
            0.209937788789,
    };

    public static double TURRET_MIN_TARGET_VELOCITY_CHANGE_REQUIRING_INTEGRAL_RESET = 20_000;

    public static TurretBasePIDFCoefficients TURRET_POSITIONAL_COEFFICIENTS = new TurretBasePIDFCoefficients(
            4,
            5,
            new double[] {0.00025, 0.00025},
            new double[] {0.007, 0.007},
            300,
            400,
            0.22,
            1200,
            new double[] {750, 750},
            1000,
            new double[] {-0.185, -0.185},
            new double[] {0, 3200},
            new double[] {0.945, 0.945},
            0.99,
            250,
            13.31,
            0.75,
            -4500,
            4500
    ).withTuning(true);

    public static double[] TURRET_KALMAN_FILTER_PARAMETERS = {1050, 200, 1800, 5};

    public static List<Double> TURRET_PD_POSITIONS = new ArrayList<>(List.of(-5800.0, 5800.0));
    public static List<Double> TURRET_KPS =          new ArrayList<>(List.of(0d, 0d));
    public static List<Double> TURRET_KDS =          new ArrayList<>(List.of(0d, 0d));
    public static List<Double> TURRET_FEEDFORWARD_POSITIONS = new ArrayList<>(List.of(-5800.0, 5800.0));
    public static List<Double> TURRET_KFS =                   new ArrayList<>(List.of(0d, 0d));

}