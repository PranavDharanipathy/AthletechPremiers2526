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

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(group = "tuning")
public class TurretActuatorTuner extends OpMode {

    public static long LOOP_TIME = 60;
    public static double KP_CLOSE = ConfigurationConstants.TURRET_VELOCITY_COEFFICIENTS[0];
    public static double KP_FAR = ConfigurationConstants.TURRET_VELOCITY_COEFFICIENTS[1];
    public static double SHARPNESS = ConfigurationConstants.TURRET_VELOCITY_COEFFICIENTS[2];
    public static double KI = ConfigurationConstants.TURRET_VELOCITY_COEFFICIENTS[3];
    public static double KI_SMASH = ConfigurationConstants.TURRET_VELOCITY_COEFFICIENTS[4];
    public static double MIN_I = ConfigurationConstants.TURRET_VELOCITY_COEFFICIENTS[5];
    public static double MAX_I = ConfigurationConstants.TURRET_VELOCITY_COEFFICIENTS[6];
    public static double KD = ConfigurationConstants.TURRET_VELOCITY_COEFFICIENTS[7];
    public static double NOMINAL_DT = ConfigurationConstants.TURRET_VELOCITY_COEFFICIENTS[8];
    public static double KV_LEFT = ConfigurationConstants.TURRET_VELOCITY_COEFFICIENTS[9];
    public static double KV_RIGHT = ConfigurationConstants.TURRET_VELOCITY_COEFFICIENTS[10];
    public static double TUNING_VOLTAGE = ConfigurationConstants.TURRET_VELOCITY_COEFFICIENTS[11];
    public static double VOLTAGE_FILTER_ALPHA = ConfigurationConstants.TURRET_VELOCITY_COEFFICIENTS[12];
    public static double KSTATIC = ConfigurationConstants.TURRET_VELOCITY_COEFFICIENTS[13];
    public static double KCOULOMB = ConfigurationConstants.TURRET_VELOCITY_COEFFICIENTS[14];
    public static double KB = ConfigurationConstants.TURRET_VELOCITY_COEFFICIENTS[15];
    public static double KSTRIBECK_VELOCITY = ConfigurationConstants.TURRET_VELOCITY_COEFFICIENTS[16];

    public static double VELOCITY;
    public static double NEGATIVE_LIMIT = -4500;
    public static double POSITIVE_LIMIT = 4500;

    private double NEGATIVE_HARD_LIMIT = -5800;
    private double POSITIVE_HARD_LIMIT = 5800;

    private TurretActuator turret;

    private Telemetry telemetry;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        turret = new TurretActuator(hardwareMap);
    }

    private double velocity = 0;
    private double currPosition = 0;
    private double prevVelocityMag, currVelocityMag = 0;
    private boolean prevSwitchVelocity;
    private boolean currSwitchVelocity = true;

    @Override
    public void loop() {

        turret.setPIDVSCoefficients(
                KP_CLOSE, KP_FAR, SHARPNESS,
                KI, KI_SMASH, MIN_I, MAX_I,
                KD, NOMINAL_DT,
                KV_LEFT, KV_RIGHT, TUNING_VOLTAGE, VOLTAGE_FILTER_ALPHA,
                KSTATIC, KCOULOMB, KB, KSTRIBECK_VELOCITY
        );

        runVelocityFlipping();
        turret.setVelocity(velocity);

        turret.update();

        telemetry.addData("kp", turret.getKp());

        telemetry.addData("kpFar", turret.kpFar);
        telemetry.addData("kpClose", turret.kpClose);

        telemetry.addData("target speed", Math.abs(turret.getTargetVelocity()));
        telemetry.addData("current speed", Math.abs(turret.getCurrentVelocity()));

        telemetry.addData("target velocity", turret.getTargetVelocity());
        telemetry.addData("current velocity", turret.getCurrentVelocity());
        telemetry.addData("velocity error", turret.getError());

        telemetry.addData("current position", turret.getCurrentPosition());
        telemetry.addData("start position", turret.getStartPosition());

        telemetry.addData("p", turret.p);
        telemetry.addData("i", turret.i);
        telemetry.addData("d", turret.d);
        telemetry.addData("v", turret.v);
        telemetry.addData("s", turret.s);

        telemetry.addData("raw derivative", turret.getRawDerivative());

        telemetry.addData("power", turret.getPower());

        telemetry.addData("dt", turret.getLoopDt());

        telemetry.update();

        sleep(LOOP_TIME);
    }

    private void runVelocityFlipping() {

        prevVelocityMag = currVelocityMag;
        currVelocityMag = VELOCITY;

        currPosition = turret.getCurrentPosition();

        prevSwitchVelocity = currSwitchVelocity;
        currSwitchVelocity = (currPosition > limitClamp(POSITIVE_LIMIT) + turret.getStartPosition()) || (currPosition < limitClamp(NEGATIVE_LIMIT) + turret.getStartPosition());

        if (currVelocityMag != prevVelocityMag) {
            velocity = currPosition >= turret.getStartPosition() ? VELOCITY : -VELOCITY;
        }

        if (currSwitchVelocity && !prevSwitchVelocity) {
            velocity*=-1;
        }
    }

    private double limitClamp(double limit) {
        return MathUtil.clamp(limit, NEGATIVE_HARD_LIMIT, POSITIVE_HARD_LIMIT);
    }
}
