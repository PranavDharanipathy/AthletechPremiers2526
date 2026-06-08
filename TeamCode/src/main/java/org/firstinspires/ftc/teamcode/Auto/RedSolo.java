package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auto.autosubsystems.IntakeNF;
import org.firstinspires.ftc.teamcode.Auto.autosubsystems.ShooterNF;
import org.firstinspires.ftc.teamcode.Constants.LocalizationConstants;
import org.firstinspires.ftc.teamcode.Systems.CurrentAlliance;
import org.firstinspires.ftc.teamcode.Systems.PoseTransfer;
import org.firstinspires.ftc.teamcode.util.PedroPathing.CancelableFollowPath;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous (name = "RedSolo", group = "A_Match", preselectTeleOp = "TeleOp_RED")
public class RedSolo extends NextFTCOpMode {

    private static final CurrentAlliance.ALLIANCE ALLIANCE = CurrentAlliance.ALLIANCE.RED_ALLIANCE;

    private Telemetry telemetry;

    private RedSoloPaths paths;

    public RedSolo() {
        addComponents(
                new SubsystemComponent(
                        RobotNF.INSTANCE,
                        IntakeNF.INSTANCE,
                        ShooterNF.INSTANCE
                ),
                new PedroComponent(LocalizationConstants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose startPose = new Pose(50.983, 41.155, Math.toRadians(-104.278)).plus(new Pose(72, 72, 0));
        PedroComponent.follower().setStartingPose(startPose);

        ShooterNF.INSTANCE.provideFollower(PedroComponent.follower());
        ShooterNF.INSTANCE.provideAlliance(ALLIANCE);

        paths = new RedSoloPaths(PedroComponent.follower(), startPose);
    }

    @Override
    public void onWaitForStart() {

        telemetry.addData("turret current position", ShooterNF.INSTANCE.turret.getCurrentPosition());
        telemetry.addData("turret start position", ShooterNF.INSTANCE.turret.startPosition);
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {

        telemetry.clearAll();
        ShooterNF.INSTANCE.start();

        //auto
        auto().schedule();
    }

    private Pose poseRecord = new Pose();

    @Override
    public void onUpdate() {

        telemetry.addData("turret target position", ShooterNF.INSTANCE.turret.getTargetPosition());
        telemetry.addData("turret current position", ShooterNF.INSTANCE.turret.getCurrentPosition());
        telemetry.addData("turret position error", ShooterNF.INSTANCE.turret.getError());

        telemetry.addData("flywheel target velocity", ShooterNF.INSTANCE.flywheel.getTargetVelocity());
        telemetry.addData("flywheel current velocity", ShooterNF.INSTANCE.flywheel.getCurrentVelocity());
        telemetry.addData("flywheel velocity error", ShooterNF.INSTANCE.flywheel.getError());

        Pose botPose = PedroComponent.follower().getPose();
        if (botPose.getX() != 0 && botPose.getY() != 0 && botPose.getHeading() != 0) {
            poseRecord = PedroComponent.follower().getPose();
        }
        telemetry.addData("bot pose", "x:%.3f, y:%.3f, heading:%.3f", poseRecord.getX(), poseRecord.getY(), Math.toDegrees(poseRecord.getHeading()));

        telemetry.update();
    }

    @Override
    public void onStop() {

        RobotNF.INSTANCE.end();

        PoseTransfer.X = poseRecord.getX();
        PoseTransfer.Y = poseRecord.getY();
        PoseTransfer.HEADING = poseRecord.getHeading();
    }

    private Command auto() {

        return new SequentialGroup(

                new FollowPath(paths.preload),

                new FollowPath(paths.firstSpikeIntake),
                new FollowPath(paths.firstSpikeReturn),

                new FollowPath(paths.secondSpikeIntake),
                new FollowPath(paths.secondSpikeReturn),

                gate(paths.firstGateIntake),
                new FollowPath(paths.firstGateReturn),

                gate(paths.secondGateIntake),
                new FollowPath(paths.secondGateReturn),

                gate(paths.thirdGateIntake),
                new FollowPath(paths.thirdGateReturn),

                new FollowPath(paths.thirdSpikeIntake),
                new FollowPath(paths.thirdSpikeReturn),

                new FollowPath(paths.hpSpikeIntake),
                new FollowPath(paths.hpSpikeReturn)
        );
    }

    private Command gate(PathChain pathChain) {

        return new SequentialGroup(

                new ParallelGroup(
                        new SequentialGroup(
                                new InstantCommand(() -> PedroComponent.follower().setMaxPower(1)),
                                new Delay(1),
                                new InstantCommand(() -> PedroComponent.follower().setMaxPower(0.8))
                        ),
                        new CancelableFollowPath(pathChain, true, 3.33).thenWait(1.5)
                ),
                new InstantCommand(() -> PedroComponent.follower().setMaxPower(1))
        );
    }

}
