package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auto.autosubsystems.IntakeNF;
import org.firstinspires.ftc.teamcode.Auto.autosubsystems.ShooterNF;
import org.firstinspires.ftc.teamcode.Constants.LocalizationConstants;
import org.firstinspires.ftc.teamcode.Systems.CurrentAlliance;
import org.firstinspires.ftc.teamcode.Systems.DataTransfer.PoseTransfer;
import org.firstinspires.ftc.teamcode.Systems.DataTransfer.TurretStartPositionTransfer;
import org.firstinspires.ftc.teamcode.util.PedroPathing.CancelableFollowPath;
import org.firstinspires.ftc.teamcode.util.PedroPathing.PowerAdjustedPath;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.conditionals.IfElseCommand;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Config
@Autonomous (name = "RedAlliance", group = "A_Match", preselectTeleOp = "TeleOp_RED")
public class RedAlliance extends NextFTCOpMode {

    public static double SHOOT_TIME = 0.2;

    public static double[] SLIP_FACTOR = {0.8, 0.47, 0.47, 0.47, 0.47, 0.47, 0.2};

    private static final CurrentAlliance.ALLIANCE ALLIANCE = CurrentAlliance.ALLIANCE.RED_ALLIANCE;

    private Telemetry telemetry;

    private RedAlliancePaths paths;

    public RedAlliance() {
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

        Pose startPose = new Pose(55.45, 39.88, Math.toRadians(0)).plus(new Pose(72, 72, 0));
        PedroComponent.follower().setStartingPose(startPose);

        ShooterNF.INSTANCE.provideFollower(PedroComponent.follower());
        ShooterNF.INSTANCE.provideAlliance(ALLIANCE);

        paths = new RedAlliancePaths(PedroComponent.follower(), startPose);
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

        TurretStartPositionTransfer.TURRET_START_POSITION = ShooterNF.INSTANCE.turret.startPosition;
        TurretStartPositionTransfer.TRANSFERRED = true;
    }

    private Command auto() {

        return new SequentialGroup(

                IntakeNF.INSTANCE.intake(),

                ShooterNF.INSTANCE.setVel(paths.preload.endPose(), 45),
                new InstantCommand(() -> ShooterNF.INSTANCE.setBallSlipFactor(SLIP_FACTOR[0])),
                new FollowPath(paths.preload, true),
                new WaitUntil(() -> Math.abs(ShooterNF.INSTANCE.flywheel.getError()) < 50),
                RobotNF.INSTANCE.shootBalls(SHOOT_TIME),

                ShooterNF.INSTANCE.setVel(paths.firstSpikeReturn.endPose(), 10),
                new InstantCommand(() -> ShooterNF.INSTANCE.setBallSlipFactor(SLIP_FACTOR[1])),
                new FollowPath(paths.firstSpikeIntake),
                new ParallelGroup(
                        RobotNF.INSTANCE.delayedIdle(0.6),
                        new FollowPath(paths.firstSpikeReturn)
                ),

                new ParallelGroup(

                        //shoot balls
                        IntakeNF.INSTANCE.intake(),
                        IntakeNF.INSTANCE.blocker(true),
                        new Delay(SHOOT_TIME),

                        new SequentialGroup(
                                new Delay(0.45),
                                IntakeNF.INSTANCE.blocker(false), //finish shooting

                                ShooterNF.INSTANCE.setVel(paths.secondSpikeReturn.endPose(), 5),
                                new InstantCommand(() -> ShooterNF.INSTANCE.setBallSlipFactor(SLIP_FACTOR[2])),
                                new FollowPath(paths.secondSpikeIntake)
                        )
                ),

                new ParallelGroup(
                        RobotNF.INSTANCE.delayedIdle(0.6),
                        cruiseShootBalls(
                                new double[] {SHOOT_TIME, 0.45},
                                1,
                                paths.secondSpikeReturn,
                                new FollowPath(paths.secondSpikeReturn),
                                new SequentialGroup(
                                        ShooterNF.INSTANCE.setVel(paths.firstGateReturn.endPose()),
                                        new InstantCommand(() -> ShooterNF.INSTANCE.setBallSlipFactor(SLIP_FACTOR[3])),
                                        gate(paths.firstGateIntake)
                                )
                        )
                ),

                new ParallelGroup(
                        RobotNF.INSTANCE.delayedIdle(0.6),
                        cruiseShootBalls(
                                new double[] {SHOOT_TIME, 0.45},
                                1,
                                paths.firstGateReturn,
                                new PowerAdjustedPath(paths.firstGateReturn, 1, 5, 0.95, 1),
                                new SequentialGroup(
                                        ShooterNF.INSTANCE.setVel(paths.secondGateReturn.endPose()),
                                        new InstantCommand(() -> ShooterNF.INSTANCE.setBallSlipFactor(SLIP_FACTOR[4])),
                                        gate(paths.secondGateIntake)
                                )
                        )
                ),

                new ParallelGroup(
                        RobotNF.INSTANCE.delayedIdle(0.6),
                        cruiseShootBalls(
                                new double[] {SHOOT_TIME, 0.45},
                                1,
                                paths.secondGateReturn,
                                new PowerAdjustedPath(paths.secondGateReturn, 1, 5, 0.95, 1),
                                new SequentialGroup(
                                        ShooterNF.INSTANCE.setVel(paths.thirdGateReturn.endPose()),
                                        new InstantCommand(() -> ShooterNF.INSTANCE.setBallSlipFactor(SLIP_FACTOR[5])),
                                        gate(paths.thirdGateIntake)
                                )
                        )
                ),

                new ParallelGroup(
                        RobotNF.INSTANCE.delayedIdle(0.6),
                        cruiseShootBalls(
                                new double[] {SHOOT_TIME, 0.45},
                                1,
                                paths.thirdGateReturn,
                                new PowerAdjustedPath(paths.thirdGateReturn, 1, 5, 0.95, 1),
                                new SequentialGroup(
                                        ShooterNF.INSTANCE.setVel(paths.fourthGateReturn.endPose(), 35),
                                        new InstantCommand(() -> ShooterNF.INSTANCE.setBallSlipFactor(SLIP_FACTOR[6])),
                                        gate(paths.fourthGateIntake)
                                )
                        )
                ),

                new ParallelGroup(
                        RobotNF.INSTANCE.delayedIdle(0.6),
                        new FollowPath(paths.fourthGateReturn),
                        RobotNF.INSTANCE.shootBalls(0.75, 8, paths.fourthGateReturn)
                ),

                new InstantCommand(ShooterNF.INSTANCE.turretToZero())
        );
    }

    private Command gate(PathChain pathChain) {

        CancelableFollowPath path = new CancelableFollowPath(pathChain, true, 2.75);

        return new SequentialGroup(

                new ParallelGroup(
                        new SequentialGroup(
                                new InstantCommand(() -> PedroComponent.follower().setMaxPower(1)),
                                new Delay(1),
                                new InstantCommand(() -> PedroComponent.follower().setMaxPower(0.775))
                        ),
                        new SequentialGroup(
                                path,
                                new IfElseCommand(path::getCancelled, new NullCommand(), new Delay(1.5))
                        )
                ),
                new InstantCommand(() -> PedroComponent.follower().setMaxPower(1))
        );
    }

    private Command cruiseShootBalls(double[] shootTime, double cruiseShootPower, PathChain shootPathChain, Command shootPath, Command nextPathCmdBlock) {

        return new SequentialGroup(

                new ParallelGroup(

                        shootPath,

                        new SequentialGroup(

                                new WaitUntil(() -> shootPathChain.lastPath().isAtParametricEnd()),
                                new InstantCommand(() -> PedroComponent.follower().setMaxPower(cruiseShootPower)),

                                IntakeNF.INSTANCE.intake(),
                                IntakeNF.INSTANCE.blocker(true),
                                new Delay(shootTime[0])
                        )
                ),

                new InstantCommand(() -> PedroComponent.follower().setMaxPower(1)),

                new ParallelGroup(
                        new Delay(shootTime[1]).then(IntakeNF.INSTANCE.blocker(false)),
                        nextPathCmdBlock
                )
        );
    }

}
