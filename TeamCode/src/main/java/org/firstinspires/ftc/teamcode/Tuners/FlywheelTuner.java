
package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.ConfigurationConstants;
import org.firstinspires.ftc.teamcode.Constants.MapSetterConstants;
import org.firstinspires.ftc.teamcode.Systems.Flywheel;
import org.firstinspires.ftc.teamcode.Systems.FlywheelPIDVSCoefficients;

@Config
@TeleOp (group = "tuning")
public class FlywheelTuner extends LinearOpMode {

    public static int LOOP_TIME = 60;

    public enum TUNING_STAGES {

        PIDVS /*1st*/, kPIDFUnitsPerVolt/*2nd*/, KV_SCALED /*3rd*/, STABILITY
    }

    public static TUNING_STAGES TUNING_STAGE = TUNING_STAGES.PIDVS;

    public static double KP_FAR = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.kpFar;
    public static double KP_CLOSE = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.kpClose;
    public static double KI_FAR = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.kiFar;
    public static double KI_CLOSE = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.kiClose;
    public static double KD = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.kd;
    public static double KV = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.unscaledKv;
    public static double KS = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.ks;
    public static double KPIDF_UNITS_PER_VOLT = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.kPIDFUnitsPerVolt;
    public static double KI_SMASH = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.kISmash;
    public static double I_SWITCH = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.iSwitch;
    public static double P_SWITCH = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.pSwitch;

    public static double D_MIN = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.minD, D_MAX = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.maxD;
    public static double I_MIN = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.minI, I_MAX = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.maxI;
    public static double P_MIN = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.minP, P_MAX = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.maxP;

    public static double VELOCITY;
    public static double VELOCITY_MARGIN_OF_ERROR = ConfigurationConstants.FLYWHEEL_VELOCITY_MARGIN_OF_ERROR;
    public static double STABILITY_MARGIN_OF_ERROR = ConfigurationConstants.FLYWHEEL_STABILITY_MARGIN_OF_ERROR;

    public static double VOLTAGE_FILTER_ALPHA = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.voltageFilterAlpha;

    public static double TOTAL_MASS_IN_GRAMS = ConfigurationConstants.FLYWHEEL_ASSEMBLY_TOTAL_WEIGHT;
    public static double SHAFT_DIAMETER = ConfigurationConstants.FLYWHEEL_SHAFT_DIAMETER;
    public static double MOTOR_CORE_VOLTAGE = ConfigurationConstants.FLYWHEEL_MOTOR_CORE_VOLTAGE;
    public static double MOTOR_RPM = ConfigurationConstants.FLYWHEEL_MOTOR_RPM;

    private Flywheel flywheel;
    private FlywheelPIDVSCoefficients coefficients;

    private VoltageSensor batteryVoltageSensor;

    private double lastVoltage;
    private double currentVoltage;

    private double lastPIDFUnits = 0;
    private double currentPIDFUnits = 0;

    @Override
    public void runOpMode() {

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.setMsTransmissionInterval(12);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        DcMotorEx leftFlywheel = hardwareMap.get(DcMotorEx.class, MapSetterConstants.leftFlywheelMotorDeviceName);
        DcMotorEx rightFlywheel = hardwareMap.get(DcMotorEx.class, MapSetterConstants.rightFlywheelMotorDeviceName);

        flywheel = new Flywheel(leftFlywheel, rightFlywheel);
        flywheel.initVoltageSensor(hardwareMap);
        flywheel.setInternalParameters(TOTAL_MASS_IN_GRAMS, SHAFT_DIAMETER, MOTOR_CORE_VOLTAGE, MOTOR_RPM);

        coefficients = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS;
        flywheel.setVelocityPIDVSCoefficients(coefficients);

        if (isStopRequested()) return;
        waitForStart();

        flywheel.reset();

        currentVoltage = batteryVoltageSensor.getVoltage();
        lastVoltage = currentVoltage;

        while (opModeIsActive()) {

            coefficients.updateCoefficients(
                    KP_FAR, KP_CLOSE,
                    KI_FAR, KI_CLOSE,
                    KD,
                    KV,
                    KS,
                    KPIDF_UNITS_PER_VOLT,
                    I_SWITCH,
                    P_SWITCH,
                    KI_SMASH,
                    VOLTAGE_FILTER_ALPHA,
                    P_MIN, P_MAX,
                    I_MIN, I_MAX,
                    D_MIN, D_MAX
            );

            lastVoltage = currentVoltage;
            currentVoltage = batteryVoltageSensor.getVoltage();

            telemetry.addData("voltage", currentVoltage);

            if (TUNING_STAGE == TUNING_STAGES.KV_SCALED) {
                coefficients.setTuning(false);
            }
            else {
                coefficients.setTuning(true);
            }

            flywheel.setVelocityPIDVSCoefficients(coefficients);

            flywheel.setVelocity(VELOCITY, true);

            flywheel.update();

            lastPIDFUnits = currentPIDFUnits;
            currentPIDFUnits = flywheel.getPIDVS()[0] + flywheel.getPIDVS()[1] + flywheel.getPIDVS()[2] + flywheel.getPIDVS()[3];

            telemetry.addData("Target Velocity", flywheel.getTargetVelocity());
            telemetry.addData("Current Velocity", flywheel.getCurrentVelocity());
            telemetry.addData("ki", flywheel.ki);
            telemetry.addData("p", flywheel.getPIDVS()[0]);
            telemetry.addData("i", flywheel.getPIDVS()[1]);
            telemetry.addData("d", flywheel.getPIDVS()[2]);
            telemetry.addData("v", flywheel.getPIDVS()[3]);
            telemetry.addData("s", flywheel.getPIDVS()[4]);

            if (TUNING_STAGE == TUNING_STAGES.STABILITY) {
                telemetry.addData("Is At Velocity And Stable", flywheel.isAtVelocityAndStable(VELOCITY_MARGIN_OF_ERROR, STABILITY_MARGIN_OF_ERROR));
            }
            else if (TUNING_STAGE == TUNING_STAGES.kPIDFUnitsPerVolt) {
                telemetry.addData("kPIDFUnitsPerVolt", Math.abs(currentPIDFUnits - lastPIDFUnits) / Math.abs(currentVoltage - lastVoltage));
            }

            telemetry.addData("Left flywheel motor power", flywheel.getMotorPowers()[0]);
            telemetry.addData("Right flywheel motor power", flywheel.getMotorPowers()[1]);
            telemetry.addData("Is motor enabled", flywheel.getMotorEnabled());
            telemetry.update();

            sleep(LOOP_TIME);
        }
    }
}
