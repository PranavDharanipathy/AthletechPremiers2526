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
public class FrictionFeedforwardStaticTuner extends OpMode {

    public static double POWER = 0;

    private TurretActuator turret;

    private Telemetry telemetry;

    @Override
    public void init() {

        telemetry  = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        turret = new TurretActuator(hardwareMap);
        turret.setPIDUsage(false);

    }

    @Override
    public void loop() {

        turret.update();

        turret.setPower(POWER);

        telemetry.addData("start position", turret.getStartPosition());
        telemetry.addData("current position", turret.getCurrentPosition());

        telemetry.update();

    }
}
