package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants.Calculations;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.TeleOp.drive.PedroDrive;
import org.firstinspires.ftc.teamcode.util.pedroPathing.PoseAcceleration;
import org.firstinspires.ftc.teamcode.util.pedroPathing.PoseVelocity;
import org.firstinspires.ftc.teamcode.util.pedroPathing.PoseSpeedTracker;

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

        poseSpeedTracker.update();

        Pose botPose = follower.getPose();

        PoseVelocity botVel = poseSpeedTracker.getPoseVelocity();
        PoseAcceleration botAccel = poseSpeedTracker.getPoseAcceleration();

        telemetry.addData("raw pose", botPose.toString());

        telemetry.addData("future pose",
                Calculations.getFutureRobotPose(
                        1.3,
                        botPose,
                        botVel,
                        ShooterConstants.THC_ACCELERATION_INFLUENCE,
                        botAccel
                ).toString()
        );

        telemetry.addData("bot vel", "x: %.2f, y: %.2f, heading: %.2f", botVel.getXVelocity(), botVel.getYVelocity(), botVel.getAngularVelocity());

        double[][] histories = poseSpeedTracker.getHistories();
        double[] xVelHistory = histories[0];
        double[] yHistory = histories[1];
        double[] angHistory = histories[2];

        telemetry.addData("xVelHistory", "OLD:%.2f, NEW:%.2f", xVelHistory[0], xVelHistory[1]);
        telemetry.addData("yVelHistory", "OLD:%.2f, NEW:%.2f", yHistory[0], yHistory[1]);
        telemetry.addData("angVelHistory", "OLD:%.2f, NEW:%.2f", angHistory[0], angHistory[1]);

        telemetry.addData("xAccelHistory", "OLD:%.2f, NEW:%.2f", xVelHistory[2], xVelHistory[3]);
        telemetry.addData("yAccelHistory", "OLD:%.2f, NEW:%.2f", yHistory[2], yHistory[3]);
        telemetry.addData("angAccelHistory", "OLD:%.2f, NEW:%.2f", angHistory[2], angHistory[3]);

        telemetry.update();

        pedroDrive.update();
    }
}
