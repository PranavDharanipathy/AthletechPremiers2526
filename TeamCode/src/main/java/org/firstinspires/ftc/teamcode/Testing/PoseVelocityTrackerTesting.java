package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants.Calculations;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.TeleOp.drive.PedroDrive;
import org.firstinspires.ftc.teamcode.util.PedroPathing.PoseAcceleration;
import org.firstinspires.ftc.teamcode.util.PedroPathing.PoseVelocity;
import org.firstinspires.ftc.teamcode.util.PedroPathing.PoseSpeedTracker;

@TeleOp(group = "testing")
public class PoseVelocityTrackerTesting extends TeleOpBaseOpMode {

    private PoseSpeedTracker poseSpeedTracker;

    private PedroDrive pedroDrive = new PedroDrive();

    @Override
    public void init() {

        initializeDevices();
        applyComponentTraits();

        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.setMsTransmissionInterval(30);

        poseSpeedTracker = new PoseSpeedTracker(follower);

        pedroDrive.provideComponents(follower, controller1);
    }

    @Override
    public void loop() {

        follower.update();
        pedroDrive.update();

        poseSpeedTracker.update();

        Pose botPose = follower.getPose();

        PoseVelocity botVel = poseSpeedTracker.getPoseVelocity();
        PoseAcceleration botAccel = poseSpeedTracker.getPoseAcceleration();

        telemetry.addData("bot pose", "x: %.2f, y: %.2f, heading: %.2f", botPose.getX(), botPose.getY(), botPose.getHeading());
        telemetry.addData("bot vel", "x: %.2f, y: %.2f, heading: %.2f", botVel.getXVelocity(), botVel.getYVelocity(), botVel.getAngularVelocity());
        telemetry.addData("bot accel", "x: %.2f, y: %.2f, heading: %.2f", botAccel.getXAcceleration(), botAccel.getYAcceleration(), botAccel.getAngularAcceleration());

        telemetry.update();

    }
}
