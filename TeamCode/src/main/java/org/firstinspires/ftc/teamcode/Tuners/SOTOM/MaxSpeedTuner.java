package org.firstinspires.ftc.teamcode.Tuners.SOTOM;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.ConfigurationConstants;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Constants.MapSetterConstants;
import org.firstinspires.ftc.teamcode.Systems.Flywheel;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.pedroPathing.PoseAcceleration;
import org.firstinspires.ftc.teamcode.util.pedroPathing.PoseSpeedTracker;
import org.firstinspires.ftc.teamcode.util.pedroPathing.PoseVelocity;

/// Gets the maximum velocity and maximum acceleration of the robot.
@Config
@Autonomous(group = "tuning")
public class MaxSpeedTuner extends LinearOpMode {

    public static boolean RUN_WITH_FLYWHEEL = false;

    private Flywheel flywheel;

    private double maxVelocity = 0.0, maxAcceleration = 0.0;

    private Follower follower;
    private PoseSpeedTracker poseSpeedTracker;

    private BetterGamepad controller1;

    private boolean keepRunning = true;

    @Override
    public void runOpMode() {

        flywheel = new Flywheel(
                hardwareMap.get(DcMotorEx.class, MapSetterConstants.leftFlywheelMotorDeviceName),
                hardwareMap.get(DcMotorEx.class, MapSetterConstants.rightFlywheelMotorDeviceName)
        );

        flywheel.initVoltageSensor(hardwareMap);
        flywheel.setInternalParameters(
                ConfigurationConstants.FLYWHEEL_ASSEMBLY_TOTAL_WEIGHT,
                ConfigurationConstants.FLYWHEEL_SHAFT_DIAMETER,
                ConfigurationConstants.FLYWHEEL_MOTOR_CORE_VOLTAGE,
                ConfigurationConstants.FLYWHEEL_MOTOR_RPM
        );
        flywheel.setVelocityPIDVSCoefficients(ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        controller1 = new BetterGamepad(gamepad1);

        follower = DriveConstants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());

        poseSpeedTracker = new PoseSpeedTracker(follower);

        follower.startTeleOpDrive(false);
        follower.update();

        telemetry.addLine("click 'a' on controller 1 to stop robot (must not hit the field barrier)");
        telemetry.update();

        if (RUN_WITH_FLYWHEEL) {
            flywheel.runMotor(Flywheel.RunningMotor.DISABLE);
            flywheel.setPower(1);
        }

        waitForStart();

        telemetry.clearAll();
        telemetry.update();

        follower.setTeleOpDrive(1, 0, 0);

        while (opModeIsActive() && keepRunning) {

            if (!controller1.a()) keepRunning = false;

            follower.update();
            poseSpeedTracker.update();

            PoseVelocity vel = poseSpeedTracker.getPoseVelocity();
            PoseAcceleration accel = poseSpeedTracker.getPoseAcceleration();

            double velTranslational = Math.hypot(vel.getXVelocity(), vel.getYVelocity());
            double accelTranslational = Math.hypot(accel.getXAcceleration(), accel.getYAcceleration());

            maxVelocity = Math.max(velTranslational, maxVelocity);
            maxAcceleration = Math.max(accelTranslational, maxAcceleration);
        }

        follower.setTeleOpDrive(0, 0, 0);

        telemetry.addLine("Use 'Max Velocity' if it was run with flywheel.");

        telemetry.addLine();

        telemetry.addData("Max Velocity", maxVelocity);
        telemetry.addData("Usable Max Velocity", maxVelocity * 0.95);

        telemetry.addLine();

        telemetry.addData("Max Acceleration", maxAcceleration);
        telemetry.addData("Usable Max Acceleration", maxAcceleration * 0.95);

        telemetry.update();

        while (opModeIsActive()) idle();
    }

}