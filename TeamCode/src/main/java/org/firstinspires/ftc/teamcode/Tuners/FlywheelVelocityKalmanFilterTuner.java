package org.firstinspires.ftc.teamcode.Tuners;

import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.ConfigurationConstants;
import org.firstinspires.ftc.teamcode.Constants.MapSetterConstants;
import org.firstinspires.ftc.teamcode.Systems.Flywheel;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.TickrateChecker;

@Config
@TeleOp (group = "tuning")
public class FlywheelVelocityKalmanFilterTuner extends OpMode {

    public static double Q = ConfigurationConstants.FLYWHEEL_KALMAN_FILTER_PARAMETERS[0];
    public static double R = ConfigurationConstants.FLYWHEEL_KALMAN_FILTER_PARAMETERS[1];
    public static double OUTLIER_SIGMA = ConfigurationConstants.FLYWHEEL_KALMAN_FILTER_PARAMETERS[2];
    public static double KR_INFLATION = ConfigurationConstants.FLYWHEEL_KALMAN_FILTER_PARAMETERS[3];

    public static long LOOP_TIME = 70;

    public static int STAGE = 0;

    public static double VELOCITY = 0;
    public static double VELOCITY_INCREMENT = 0.005;
    public static double MAX_VELOCITY = 520_000;
    private int velDirection = 1;

    private double vel;

    private Flywheel flywheel;

    private Telemetry telemetry;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        flywheel = new Flywheel(
                hardwareMap.get(DcMotorEx.class, MapSetterConstants.leftFlywheelMotorDeviceName),
                hardwareMap.get(DcMotorEx.class, MapSetterConstants.rightFlywheelMotorDeviceName)
        );

        flywheel.initVoltageSensor(hardwareMap);

        flywheel.setInternalParameters(
                ConfigurationConstants.FLYWHEEL_ASSEMBLY_TOTAL_WEIGHT,
                ConfigurationConstants.FLYWHEEL_SHAFT_DIAMETER,
                ConfigurationConstants.FLYWHEEL_MOTOR_CORE_VOLTAGE,
                ConfigurationConstants.FLYWHEEL_MOTOR_RPM
        );

        flywheel.setVelocityPIDVSCoefficients(ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS);
    }

    @Override
    public void loop() {

        double dt = TickrateChecker.getTimePerTick();

        // updates constants
        flywheel.getEncoder().setupVelocityKalmanFilter(new double[] {Q, R, OUTLIER_SIGMA, KR_INFLATION});

        switch (STAGE) {

            case 0:

                vel = VELOCITY;
                break;

            case 1:

                vel +=(VELOCITY_INCREMENT * velDirection);
                vel = MathUtil.clamp(vel, 0, MAX_VELOCITY);

                if (vel == 0 || vel == MAX_VELOCITY) velDirection *=-1;
                break;
        }

        flywheel.setVelocity(vel, STAGE == 0);

        flywheel.update();

        telemetry.addData("estimated velocity", flywheel.getVelocityEstimate());
        telemetry.addData("real velocity", flywheel.getEncoder().getFilteredVelocity());

        telemetry.addData("dt", dt);
        telemetry.update();

        sleep(LOOP_TIME);

    }
}