package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Systems.TurretActuator;

@Config
@TeleOp(group = "testing")
public class TurretActuatorTesting extends OpMode {

    public static double POWER_MAG = 0;
    public static double NEGATIVE_LIMIT = -4500;
    public static double POSITIVE_LIMIT = 4500;

    private TurretActuator turret;

    private Telemetry telemetry;

    private double startPosition;

    @Override
    public void init() {

        telemetry  = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        turret = new TurretActuator(hardwareMap);
        turret.setPIDUsage(false);

        startPosition = turret.getStartPosition();
    }
    private double power = 0;

    private double prevPosition, currPosition = 0;

    private double prevPowerMag, currPowerMag = 0;

    private boolean prevSwitchPower;
    private boolean currSwitchPower = true;

    @Override
    public void loop() {

        prevPowerMag = currPowerMag;
        currPowerMag = POWER_MAG;

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

        telemetry.addData("position", turret.getCurrentPosition());
        telemetry.addData("start position", turret.getStartPosition());
        telemetry.addData("power", turret.getPower());

        telemetry.update();

    }
}
