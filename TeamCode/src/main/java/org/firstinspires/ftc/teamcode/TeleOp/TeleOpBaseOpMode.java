package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.CameraConstants;
import org.firstinspires.ftc.teamcode.Constants.ConfigurationConstants;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.Constants.HoodConstants;
import org.firstinspires.ftc.teamcode.Constants.MapSetterConstants;
import org.firstinspires.ftc.teamcode.Systems.Blocker;
import org.firstinspires.ftc.teamcode.Systems.Camera;
import org.firstinspires.ftc.teamcode.Systems.Flywheel;
import org.firstinspires.ftc.teamcode.Systems.HoodAngler;
import org.firstinspires.ftc.teamcode.Systems.Lift;
import org.firstinspires.ftc.teamcode.Systems.TurretBase;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.GeneralVeloMotor;

import java.util.List;

public abstract class TeleOpBaseOpMode extends OpMode {

    public TeleOpBaseOpMode() {}

    public BetterGamepad controller1;
    public BetterGamepad controller2;

    public DcMotor left_front, right_front, left_back, right_back;

    public GeneralVeloMotor transfer;
    public DcMotor intake;
    public Blocker blocker;
    public Lift lift;

    public Camera camera;

    public Follower follower;

    public TurretBase turret;

    public HoodAngler hoodAngler;

    public Flywheel flywheel;

    private List<LynxModule> robotHubs;

    /// initializes/creates LynxModule
    public void setUpLynxModule() {

        robotHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : robotHubs) {
            hub.setBulkCachingMode(MapSetterConstants.bulkCachingMode);
        }
    }

    /// Clears cache of LynxModule
    public void clearCacheOfLynxModule() {

        for (LynxModule hub : robotHubs) {
            hub.clearBulkCache();
        }
    }

    /// Initializing devices
    public void initializeDevices() {

        left_front = hardwareMap.get(DcMotor.class, MapSetterConstants.leftFrontMotorDeviceName);
        right_front = hardwareMap.get(DcMotor.class, MapSetterConstants.rightFrontMotorDeviceName);
        left_back = hardwareMap.get(DcMotor.class, MapSetterConstants.leftBackMotorDeviceName);
        right_back = hardwareMap.get(DcMotor.class, MapSetterConstants.rightBackMotorDeviceName);

        follower = DriveConstants.createFollower(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, MapSetterConstants.intakeMotorDeviceName);

        transfer = new GeneralVeloMotor(hardwareMap, MapSetterConstants.transferMotorDeviceName);

        blocker = new Blocker(hardwareMap.get(Servo.class, MapSetterConstants.blockerServoDeviceName));

        lift = new Lift(hardwareMap.get(Servo.class, MapSetterConstants.liftServoDeviceName));

        camera = new Camera(follower, Camera.from(hardwareMap, MapSetterConstants.limelight3AUSBDeviceName));

        flywheel = new Flywheel(
                hardwareMap.get(DcMotorEx.class, MapSetterConstants.leftFlywheelMotorDeviceName),
                hardwareMap.get(DcMotorEx.class, MapSetterConstants.rightFlywheelMotorDeviceName)
        );

        turret = new TurretBase(hardwareMap);

        hoodAngler = new HoodAngler(hardwareMap, MapSetterConstants.hoodAnglerServoDeviceName);

        controller1 = new BetterGamepad(gamepad1);
        controller2 = new BetterGamepad(gamepad2);
    }

    /// Provide traits
    public void applyComponentTraits() {

        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower.setPose(FieldConstants.RELOCALIZATION_POSE);

        follower.update();

        hoodAngler.setEncoderCalibration(
                HoodConstants.HOOD_ANALOG_ENCODER_VOLTAGE_RANGE[0],
                HoodConstants.HOOD_ANALOG_ENCODER_VOLTAGE_RANGE[1]
        );

        intake.setDirection(ConfigurationConstants.INTAKE_MOTOR_DIRECTION);
        transfer.setDirection(ConfigurationConstants.TRANSFER_MOTOR_DIRECTION);

        transfer.setVelocityPDFCoefficients(
                ConfigurationConstants.TRANSFER_PDF_COEFFICIENTS[0],
                ConfigurationConstants.TRANSFER_PDF_COEFFICIENTS[1],
                ConfigurationConstants.TRANSFER_PDF_COEFFICIENTS[2]
        );

        camera.setPollRateHz(CameraConstants.CAMERA_POLL_RATE);

        flywheel.initVoltageSensor(hardwareMap);
        flywheel.setInternalParameters(
                ConfigurationConstants.FLYWHEEL_ASSEMBLY_TOTAL_WEIGHT,
                ConfigurationConstants.FLYWHEEL_SHAFT_DIAMETER,
                ConfigurationConstants.FLYWHEEL_MOTOR_CORE_VOLTAGE,
                ConfigurationConstants.FLYWHEEL_MOTOR_RPM
        );
        flywheel.setVelocityPIDVSCoefficients(ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS);

        turret.setPIDFSCoefficients(ConfigurationConstants.TURRET_PIDFS_COEFFICIENTS);
        //turret.reverse();

    }

}