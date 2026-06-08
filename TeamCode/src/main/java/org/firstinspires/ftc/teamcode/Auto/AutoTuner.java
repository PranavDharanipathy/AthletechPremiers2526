package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.LocalizationConstants;

@Config
@TeleOp(group = "tuning")
public class AutoTuner extends OpMode {

    public static double START_X = 50.983;
    public static double START_Y = 41.155;
    public static double START_HEADING_DEG = -104.278;
    public static boolean START_FROM_CENTER = true;

    private Follower follower;

    private Telemetry telemetry;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = LocalizationConstants.createFollower(hardwareMap);

        Pose startPose = new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG));
        if (START_FROM_CENTER) startPose = startPose.plus(new Pose(72, 72, 0));

        follower.setStartingPose(startPose);
        follower.update();
    }

    @Override
    public void loop() {

        follower.update();

        Pose botPose = follower.getPose();

        telemetry.addData("Pose", "x:%.3f, y:%.3f, heading:%.3f", botPose.getX(), botPose.getY(), Math.toDegrees(botPose.getHeading()));
        telemetry.update();
    }
}
