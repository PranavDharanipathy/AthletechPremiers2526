package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants.Calculations;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.TeleOp.drive.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.util.pedroPathing.PoseVelocity;
import org.firstinspires.ftc.teamcode.util.pedroPathing.PoseVelocityTracker;

@TeleOp(group = "testing")
public class PoseVelocityTrackerTesting extends TeleOpBaseOpMode {

    private PoseVelocityTracker poseVelocityTracker;

    private RobotCentricDrive robotCentricDrive = new RobotCentricDrive();

    @Override
    public void init() {

        initializeDevices();
        applyComponentTraits();

        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.setMsTransmissionInterval(30);

        poseVelocityTracker = new PoseVelocityTracker(follower);

        robotCentricDrive.provideComponents(left_front, right_front, left_back, right_back, controller1);
    }

    @Override
    public void loop() {

        follower.update();

        poseVelocityTracker.update();

        Pose botPose = follower.getPose();

        PoseVelocity botVel = poseVelocityTracker.getPoseVelocity();

        telemetry.addData("raw pose", botPose.toString());

        telemetry.addData("future pose",
                Calculations.getFutureRobotPose(
                        1.3,
                        botPose,
                        botVel
                ).toString()
        );

        telemetry.addData("bot vel", "x: %.2f, y: %.2f, heading: %.2f", botVel.getXVelocity(), botVel.getYVelocity(), botVel.getAngularVelocity());

        double[][] histories = poseVelocityTracker.getHistories();
        double[] xVelHistory = histories[0];
        double[] yVelHistory = histories[1];
        double[] angVelHistory = histories[2];

        telemetry.addData("xVelHistory", "OLD:%.2f, NEW:%.2f", xVelHistory[0], xVelHistory[1]);
        telemetry.addData("yVelHistory", "OLD:%.2f, NEW:%.2f", yVelHistory[0], yVelHistory[1]);
        telemetry.addData("angVelHistory", "OLD:%.2f, NEW:%.2f", angVelHistory[0], angVelHistory[1]);

        telemetry.update();

        robotCentricDrive.update();
    }
}
