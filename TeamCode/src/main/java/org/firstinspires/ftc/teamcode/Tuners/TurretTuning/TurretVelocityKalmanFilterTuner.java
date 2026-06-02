package org.firstinspires.ftc.teamcode.Tuners.TurretTuning;

import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.ConfigurationConstants;
import org.firstinspires.ftc.teamcode.Systems.TurretActuator;
import org.firstinspires.ftc.teamcode.util.MathUtil;

@Config
@TeleOp(group = "tuning")
public class TurretVelocityKalmanFilterTuner extends OpMode {

    public static double Q = ConfigurationConstants.TURRET_KALMAN_FILTER_PARAMETERS[0];
    public static double R = ConfigurationConstants.TURRET_KALMAN_FILTER_PARAMETERS[1];
    public static double OUTLIER_SIGMA = ConfigurationConstants.TURRET_KALMAN_FILTER_PARAMETERS[2];
    public static double KR_INFLATION = ConfigurationConstants.TURRET_KALMAN_FILTER_PARAMETERS[3];

    public static long LOOP_TIME = 60;

    public static double NEGATIVE_LIMIT = -4500;
    public static double POSITIVE_LIMIT = 4500;

    public static int STAGE = 0;

    public static double POWER_MAG = 0;
    public static double POWER_INCREMENT = 0.005;

    public static double MIN_POWER = -1;
    public static double MAX_POWER = 1;
    private int powerDirection = 1;

    private double targetPower;

    private TurretActuator turret;

    private Telemetry telemetry;

    private double startPosition;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        turret = new TurretActuator(hardwareMap);
        turret.setPIDVSCoefficients(0,0,0,0,0,-1,1,0,0,0, 0, 0, 0,0,0,0,0);

        startPosition = turret.getStartPosition();
    }

    private double prevPosition, currPosition = 0;

    private double power;

    private double prevPowerMag, currPowerMag = 0;

    private boolean prevSwitchPower;
    private boolean currSwitchPower = true;

    @Override
    public void loop() {

        // updates constants
        turret.getEncoder().setupVelocityKalmanFilter(new double[] {Q, R, OUTLIER_SIGMA, KR_INFLATION});
        turret.update();

        prevPosition = currPosition;
        currPosition = turret.getCurrentPosition();

        switch (STAGE) {

            case 0:

                targetPower = POWER_MAG;
                break;

            case 1:

                targetPower +=(POWER_INCREMENT * turret.getLoopDt() * powerDirection);
                targetPower = MathUtil.clamp(targetPower, MIN_POWER, MAX_POWER);

                if (targetPower == MIN_POWER || targetPower == MAX_POWER) powerDirection *=-1;
                break;
        }

        prevPowerMag = currPowerMag;
        currPowerMag = targetPower;

        prevSwitchPower = currSwitchPower;
        currSwitchPower = (currPosition > POSITIVE_LIMIT + startPosition) || (currPosition < NEGATIVE_LIMIT + startPosition);

        if (currPowerMag != prevPowerMag) {
            power = currPosition >= startPosition ? POWER_MAG : -POWER_MAG;
        }

        if (currSwitchPower && !prevSwitchPower) {
            power*=-1;
        }

        turret.setPower(power);

        telemetry.addData("estimated velocity", turret.getVelocityEstimate());
        telemetry.addData("real velocity", turret.getEncoder().getFilteredVelocity());

        telemetry.addData("current position", turret.getCurrentPosition());
        telemetry.addData("start position", turret.getStartPosition());

        telemetry.addData("dt", turret.getLoopDt());

        telemetry.addData("left power", turret.getServoPowers()[0]);
        telemetry.addData("right power", turret.getServoPowers()[1]);

        telemetry.update();

        sleep(LOOP_TIME);

    }
}