package org.firstinspires.ftc.teamcode.Tuners.TurretTuning.TurretActuatorFrictionFeedforwardTuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Systems.TurretActuator;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(group = "tuning")
public class FrictionFeedforwardStribeckVelocityTuner extends OpMode {

    public static double POWER_RAMP = 0.01;
    public static double INCREMENT_TIME_STATIONARY = 1;
    public static double INCREMENT_TIME_MOVING = 0.65;
    public static double VELOCITY_EPSILON = 4000;

    private final double KE = 12d / (6000d * ShooterConstants.TURRET_TICKS_PER_DEGREE);

    private TurretActuator turret;

    private VoltageSensor batteryVoltageSensor;

    private Telemetry telemetry;

    private BetterGamepad controller1;

    @Override
    public void init() {

        telemetry  = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        controller1 = new BetterGamepad(gamepad1);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        turret = new TurretActuator(hardwareMap);
        turret.setPIDUsage(false);

    }

    private ElapsedTime timer = new ElapsedTime();

    private final List<String> data = new ArrayList<>();

    @Override
    public void start() {
        timer.reset();
    }

    private double power = 0;
    private boolean running = true;

    @Override
    public void loop() {

        controller1.getInformation();
        turret.update();

        if (controller1.bHasJustBeenPressed) running = false;

        double incrementTime = turret.getCurrentVelocity() > VELOCITY_EPSILON ? INCREMENT_TIME_MOVING : INCREMENT_TIME_STATIONARY;

        if (running && timer.seconds() >= incrementTime) {

            double currentMotorPower = turret.getPower();

            double appliedVoltage = currentMotorPower * batteryVoltageSensor.getVoltage();
            double velocity = turret.getCurrentVelocity();
            double torqueProxy = 2d /*because we use 2 servos*/ * appliedVoltage - (KE * velocity);

            data.add("(" + velocity + ", " + torqueProxy + ")");

            if (Math.abs(currentMotorPower) >= 1) {
                running = true;
            }

            power += POWER_RAMP;
            timer.reset();
        }

        if (!running) turret.setPower(0);
        else turret.setPower(power);

        if (!data.isEmpty()) telemetry.addData("data", data);

        telemetry.addData("power", power);
        telemetry.addData("incrementTime", incrementTime);

        telemetry.addData("start position", turret.getStartPosition());
        telemetry.addData("current position", turret.getCurrentPosition());
        telemetry.addData("current power", turret.getPower());
        telemetry.addData("current velocity", turret.getCurrentVelocity());

        if (!running) {
            telemetry.addLine("The ωs produced may need to be edited.");
            telemetry.addLine("If turret stutters near zero speed → ωs too small (decay too fast)");
            telemetry.addLine("If turret feels mushy / sluggish leaving rest → ωs too large (decay too slow)");
            telemetry.addLine("F=Fmin+(e^-1)(Fmax−Fmin), the velocity input that leads to F is ωs");
        }

        telemetry.update();

    }
}
