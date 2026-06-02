package org.firstinspires.ftc.teamcode.Tuners.TurretTuning.TurretActuatorFrictionFeedforwardTuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Systems.TurretActuator;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.MathUtil;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(group = "tuning")
public class FrictionFeedforwardBAndCoulombTuner extends OpMode {

    public static double POWER_MAG = 0;
    public static double NEGATIVE_LIMIT = -4500;
    public static double POSITIVE_LIMIT = 4500;

    private TurretActuator turret;

    private Telemetry telemetry;

    private BetterGamepad controller1;

    private double startPosition;

    @Override
    public void init() {

        telemetry  = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        controller1 = new BetterGamepad(gamepad1);

        turret = new TurretActuator(hardwareMap);
        turret.setPIDUsage(false);

        startPosition = turret.getStartPosition();

        telemetry.addLine("Get 5 to 6 (ω, u) values.");
        telemetry.addLine("Plot them and produce a linear regression from the values.");
        telemetry.addLine("kB is the slope of the function");
        telemetry.addLine("kCoulomb is the y-intercept of the function");
        telemetry.update();
    }
    private double power;

    private double prevPosition, currPosition = 0;

    private double velocityReading = 0;
    private final List<Double> averageVelocity = new ArrayList<>();

    private double prevPowerMag, currPowerMag = 0;

    private boolean prevSwitchPower;
    private boolean currSwitchPower = true;

    @Override
    public void start() {
        telemetry.clearAll();
        telemetry.update();
    }

    @Override
    public void loop() {

        prevPowerMag = currPowerMag;
        currPowerMag = POWER_MAG;

        controller1.getInformation();

        turret.update();

        prevPosition = currPosition;
        currPosition = turret.getCurrentPosition();

        prevSwitchPower = currSwitchPower;
        currSwitchPower = (currPosition > POSITIVE_LIMIT + startPosition) || (currPosition < NEGATIVE_LIMIT + startPosition);

        if (currPowerMag != prevPowerMag) {
            power = currPosition >= startPosition ? POWER_MAG : -POWER_MAG;
        }

        if (currSwitchPower && !prevSwitchPower) {
            power*=-1;
        }

        turret.setPower(power);

        if (controller1.aHasJustBeenPressed) {
            velocityReading = Math.abs(turret.getCurrentVelocity());
            averageVelocity.add(velocityReading);
        }

        if (controller1.bHasJustBeenPressed) averageVelocity.clear();

        telemetry.addData("velocity", turret.getCurrentVelocity());
        telemetry.addData("position", turret.getCurrentPosition());

        if (!averageVelocity.isEmpty()) {
            telemetry.addData("data recorded at point", "ω:%.8f u:%.8f", velocityReading, POWER_MAG);
            telemetry.addData("vel data raw", averageVelocity);
        }

        telemetry.addData("power", turret.getPower());


        telemetry.update();

    }
}
