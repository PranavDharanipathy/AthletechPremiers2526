package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.CameraConstants;
import org.firstinspires.ftc.teamcode.Systems.CurrentAlliance;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.TeleOp.drive.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.util.BooleanTrigger;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.TickrateChecker;

@Config
@TeleOp(group = "testing")
public class CameraTesting extends TeleOpBaseOpMode {

    public static CurrentAlliance.ALLIANCE ALLIANCE = CurrentAlliance.ALLIANCE.BLUE_ALLIANCE;

    private RobotCentricDrive robotCentricDrive = new RobotCentricDrive();

    private Telemetry telemetry;

    @Override
    public void init() {

        initializeDevices();
        applyComponentTraits();

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(30);

        robotCentricDrive.provideComponents(left_front, right_front, left_back, right_back, controller1);
        
        camera.pipelineSwitch(CameraConstants.PIPELINES.getPipelineFromAlliance(ALLIANCE).getPipelineIndex());
    }

    @Override
    public void start() {

        follower.setPose(new Pose(0,0,Math.toRadians(90)));

        camera.start();
    }

    @Override
    public void loop() {

        telemetry.addData("Loop time", TickrateChecker.getTimePerTick());

        controller1.getInformation();
        controller2.getInformation();

        follower.update();
        camera.update(new BooleanTrigger(controller2.main_buttonHasJustBeenPressed));

        robotCentricDrive.update();

        telemetry.addData("Localize Pose", tfp(camera.getLocalizePose()));
        telemetry.addData("LL Pose", tfp(camera.getBotPoseMT2()));
        telemetry.addData("Follower Pose", tfp(follower.getPose()));

        Pose finalPose = camera.isEligibleForMT2() ? camera.getBotPoseMT2() : follower.getPose();

        telemetry.addData("Final Pose", finalPose);

        telemetry.addData("isRobotOrientationUpdated", camera.isRobotOrientationUpdated());
        telemetry.addData("isEligibleForMT2", camera.isEligibleForMT2());
        telemetry.addData("MT1 Localization Outcome", camera.getMt1LocalizationOutcome());

        telemetry.update();

    }

    private String tfp(Pose pose) { // telemetry friendly pose

        if (pose == null) return "NULL";

        return "x: " + MathUtil.truncate(pose.getX(), 100) +
                ", y: " + MathUtil.truncate(pose.getY(), 100) +
                ", heading: " + Math.toDegrees(MathUtil.truncate(pose.getHeading(), 100));
    }
}
