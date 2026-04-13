package org.firstinspires.ftc.teamcode.Tuners.DriveTuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.ConfigurationConstants;
import org.firstinspires.ftc.teamcode.Systems.FlywheelPIDVSCoefficients;
import org.firstinspires.ftc.teamcode.TeleOp.PostAutonomousRobotReset;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpBaseOpMode;

@Config
@TeleOp (group = "tuning")
public class FlywheelDriveTuning extends TeleOpBaseOpMode {

    public static double KP_FAR = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.kpFar;
    public static double KP_CLOSE = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.kpClose;
    public static double KI_FAR = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.kiFar;
    public static double KI_CLOSE = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.kiClose;
    public static double KD = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.kd;
    public static double KV = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.unscaledKv;
    public static double KS = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.ks;
    public static double KPIDF_UNITS_PER_VOLT = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.kPIDFUnitsPerVolt;
    public static double I_SWITCH = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.iSwitch;
    public static double P_SWITCH = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.pSwitch;
    public static double KI_SMASH = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.kISmash;
    public static double VOLTAGE_FILTER_ALPHA = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.voltageFilterAlpha;
    public static double D_MIN = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.minD, D_MAX = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.maxD;
    public static double I_MIN = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.minI, I_MAX = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.maxI;
    public static double P_MIN = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.minP, P_MAX = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.maxP;

    public static double TRANSFER_VELOCITY = 1850;
    public static double INTAKE_POWER = 1;
    public static double FLYWHEEL_VELOCITY = 0;
    public static double HOOD_POSITION = 0.2;

    private Telemetry telemetry;

    private FlywheelPIDVSCoefficients coefficients = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        initializeDevices();

        applyComponentTraits();

        //setup lynx module
        setUpLynxModule();
    }

    @Override
    public void start() {
        flywheel.setVelocityPIDVSCoefficients(coefficients);
        new PostAutonomousRobotReset(this);
    }

    @Override
    public void loop() {

        clearCacheOfLynxModule();

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

        flywheel.setVelocityPIDVSCoefficients(coefficients);

        hoodAngler.setPosition(HOOD_POSITION);
        intake.setPower(INTAKE_POWER);
        transfer.setVelocity(TRANSFER_VELOCITY);
        flywheel.setVelocity(FLYWHEEL_VELOCITY, true);
        transfer.update();
        flywheel.update();

        telemetry.addData("Target Velocity", flywheel.getTargetVelocity());
        telemetry.addData("Real Velocity", flywheel.getCurrentVelocity());
        telemetry.addData("p", "%.5f", flywheel.p);
        telemetry.addData("i", "%.5f", flywheel.i);
        telemetry.addData("d", "%.5f", flywheel.d);
        telemetry.addData("v", "%.5f", flywheel.v);

        telemetry.addData("flywheel power", "%.5f", flywheel.getPower());
        telemetry.update();
    }

}