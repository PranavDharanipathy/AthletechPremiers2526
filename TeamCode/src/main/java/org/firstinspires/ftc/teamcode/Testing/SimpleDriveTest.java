package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Constants.MapSetterConstants;
import org.firstinspires.ftc.teamcode.Systems.Blocker;
import org.firstinspires.ftc.teamcode.Systems.HoodAngler;
import org.firstinspires.ftc.teamcode.Systems.Intake;

import org.firstinspires.ftc.teamcode.TeleOp.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.TeleOp.drive.RobotCentricDrive;

@TeleOp (group = "testing")
public class SimpleDriveTest extends TeleOpBaseOpMode {

    private final Intake intake = new Intake();
    private final Blocker blocker = new Blocker().asSubsystem();
    private final RobotCentricDrive drive = new RobotCentricDrive();

    @Override
    public void init() {

        initializeDevices();

        hoodAngler = new HoodAngler(hardwareMap, MapSetterConstants.hoodAnglerServoDeviceName);

        applyComponentTraits();

        DcMotor[] motors = {
                hardwareMap.dcMotor.get(MapSetterConstants.leftFrontMotorDeviceName),
                hardwareMap.dcMotor.get(MapSetterConstants.rightFrontMotorDeviceName),
                hardwareMap.dcMotor.get(MapSetterConstants.leftBackMotorDeviceName),
                hardwareMap.dcMotor.get(MapSetterConstants.rightBackMotorDeviceName)
        };

        motors[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[2].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[3].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[2].setDirection(DcMotorSimple.Direction.REVERSE);

        drive.provideComponents(
                motors[0],
                motors[1],
                motors[2],
                motors[3],
                controller1
        );
        intake.provideComponents(super.intake, blocker /*subsystem*/, controller1);
        blocker.provideComponents(super.blocker, controller1);
        setUpLynxModule();
    }

    private HoodAngler hoodAngler;

    @Override
    public void loop() {

        clearCacheOfLynxModule();

        controller1.getInformation();

        blocker.update();
        intake.update();
        hood();
        flywheel();
        drive.update();

    }

    private double hoodPosition = 0.9;
    private void hood() {

        if (controller1.dpad_upHasJustBeenPressed) {
            hoodPosition+=0.005;
        }
        else if (controller1.dpad_downHasJustBeenPressed) {
            hoodPosition-=0.005;
        }

        hoodAngler.setPosition(hoodPosition);
    }

    private double flywheelVelocity = 1700;

    private void flywheel() {

        if (controller1.yHasJustBeenPressed) {
            flywheelVelocity+=100;
        }
        else if (controller1.aHasJustBeenPressed) {
            flywheelVelocity-=100;
        }

        flywheel.setVelocity(flywheelVelocity, true);
        flywheel.update();
    }
}