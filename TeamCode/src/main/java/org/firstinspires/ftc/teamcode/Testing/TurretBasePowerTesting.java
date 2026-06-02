package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.Constants.ConfigurationConstants;
import org.firstinspires.ftc.teamcode.Constants.MapSetterConstants;

@Config
@TeleOp (group = "testing")
public class TurretBasePowerTesting extends OpMode {

    private CRServoImplEx leftTurretBase;
    private CRServoImplEx rightTurretBase;

    public static double LEFT_POWER = 0;
    public static double RIGHT_POWER = 0;

    @Override
    public void init() {

        leftTurretBase = hardwareMap.get(CRServoImplEx.class, MapSetterConstants.turretBaseLeftServoDeviceName);
        rightTurretBase = hardwareMap.get(CRServoImplEx.class, MapSetterConstants.turretBaseRightServoDeviceName);

        leftTurretBase.setPwmEnable();
        rightTurretBase.setPwmEnable();

        leftTurretBase.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rightTurretBase.setPwmRange(new PwmControl.PwmRange(500, 2500));

        leftTurretBase.setDirection(ConfigurationConstants.TURRET_BASE_DIRECTIONS[0]);
        rightTurretBase.setDirection(ConfigurationConstants.TURRET_BASE_DIRECTIONS[1]);
    }

    @Override
    public void loop() {

        leftTurretBase.setPower(LEFT_POWER);
        rightTurretBase.setPower(RIGHT_POWER);
    }
}
