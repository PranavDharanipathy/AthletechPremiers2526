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
import org.firstinspires.ftc.teamcode.util.PedroPathing.PowerAdjustedPath;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.conditionals.IfElseCommand;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.NullCommand;
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
        ShooterNF.INSTANCE.startShooter();

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

                IntakeNF.INSTANCE.intake(),

                ShooterNF.INSTANCE.setVel(paths.preload.lastPath().endPose()),
                new FollowPath(paths.preload),
                RobotNF.INSTANCE.shootBalls(0.8, paths.preload),

                ShooterNF.INSTANCE.setVel(paths.firstSpikeIntake.lastPath().endPose()),
                new FollowPath(paths.firstSpikeIntake),
                new PowerAdjustedPath(paths.firstSpikeReturn, 1, 10, 0.65, 1),
                RobotNF.INSTANCE.shootBalls(0.8, paths.preload),

                ShooterNF.INSTANCE.setVel(paths.secondSpikeIntake.lastPath().endPose()),
                new FollowPath(paths.secondSpikeIntake),
                new PowerAdjustedPath(paths.secondSpikeReturn, 1, 10, 0.65, 1),
                RobotNF.INSTANCE.shootBalls(0.8, paths.preload),

                ShooterNF.INSTANCE.setVel(paths.firstGateIntake.lastPath().endPose()),
                gate(paths.firstGateIntake),
                new FollowPath(paths.firstGateReturn),
                RobotNF.INSTANCE.shootBalls(0.8, paths.preload),

                ShooterNF.INSTANCE.setVel(paths.secondGateIntake.lastPath().endPose()),
                gate(paths.secondGateIntake),
                new FollowPath(paths.secondGateReturn),
                RobotNF.INSTANCE.shootBalls(0.8, paths.preload),

                ShooterNF.INSTANCE.setVel(paths.thirdGateIntake.lastPath().endPose()),
                gate(paths.thirdGateIntake),
                new FollowPath(paths.thirdGateReturn),
                RobotNF.INSTANCE.shootBalls(0.8, paths.preload),

                ShooterNF.INSTANCE.setVel(paths.thirdSpikeIntake.lastPath().endPose()),
                new FollowPath(paths.thirdSpikeIntake),
                new PowerAdjustedPath(paths.thirdSpikeReturn, 1, 10, 0.65, 1),
                RobotNF.INSTANCE.shootBalls(0.8, paths.preload),

                ShooterNF.INSTANCE.setVel(paths.hpSpikeIntake.lastPath().endPose()),
                new FollowPath(paths.hpSpikeIntake),
                new PowerAdjustedPath(paths.hpSpikeReturn, 1, 12, 0.6, 1),
                RobotNF.INSTANCE.shootBalls(0.8, paths.preload)
        );
    }

    private Command gate(PathChain pathChain) {

        CancelableFollowPath path = new CancelableFollowPath(pathChain, true, 2.75);

        return new SequentialGroup(

                new ParallelGroup(
                        new SequentialGroup(
                                new InstantCommand(() -> PedroComponent.follower().setMaxPower(1)),
                                new Delay(1),
                                new InstantCommand(() -> PedroComponent.follower().setMaxPower(0.7))
                        ),
                        new SequentialGroup(
                                path,
                                new IfElseCommand(path::getCancelled, new NullCommand(), new Delay(1.5))
                        )
                ),
                new InstantCommand(() -> PedroComponent.follower().setMaxPower(1))
        );
    }

}
