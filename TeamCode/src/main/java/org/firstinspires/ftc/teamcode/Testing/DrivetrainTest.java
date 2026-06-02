package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Constants.MapSetterConstants;
import org.firstinspires.ftc.teamcode.TeleOp.drive.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;

@TeleOp (group = "testing")
public class DrivetrainTest extends OpMode {

    private RobotCentricDrive drive = new RobotCentricDrive();

    private BetterGamepad controller1;

    @Override
    public void init() {

        controller1 = new BetterGamepad(gamepad1);

        DcMotor[] motors = {
                hardwareMap.dcMotor.get(MapSetterConstants.leftFrontMotorDeviceName),
                hardwareMap.dcMotor.get(MapSetterConstants.rightFrontMotorDeviceName),
                hardwareMap.dcMotor.get(MapSetterConstants.leftBackMotorDeviceName),
                hardwareMap.dcMotor.get(MapSetterConstants.rightBackMotorDeviceName)
        };

        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[2].setDirection(DcMotorSimple.Direction.REVERSE);

        drive.provideComponents(
                motors[0],
                motors[1],
                motors[2],
                motors[3],
                controller1
        );

    }

    @Override
    public void loop() {

        controller1.getInformation();

        drive.update();

    }
}
