package org.firstinspires.ftc.teamcode.Auto.Red;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auto.RobotNF;
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
@Autonomous (name = "RedSolo", group = "A_Match", preselectTeleOp = "TeleOp_RED")
public class RedSolo extends NextFTCOpMode {

    public static double SHOOT_TIME = 0.35;

    public static double[] SLIP_FACTOR = {0.7, 0.198, 0.43, 0.4, 0.4, 0.4, 0.2};

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
                new PedroComponent(LocalizationConstants::createHardFollower),
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

        TurretStartPositionTransfer.TURRET_START_POSITION = ShooterNF.INSTANCE.turret.startPosition;
        TurretStartPositionTransfer.TRANSFERRED = true;
    }

    private Command auto() {

        return new SequentialGroup(

                IntakeNF.INSTANCE.intake(),

                ShooterNF.INSTANCE.setVel(paths.preload.endPose(), 40),
                new InstantCommand(() -> ShooterNF.INSTANCE.setBallSlipFactor(SLIP_FACTOR[0])),
                new FollowPath(paths.preload, true),
                new WaitUntil(() -> Math.abs(ShooterNF.INSTANCE.flywheel.getError()) < 50),
                RobotNF.INSTANCE.shootBalls(SHOOT_TIME),

                ShooterNF.INSTANCE.setVel(paths.firstSpikeReturn.endPose()),
                new InstantCommand(() -> ShooterNF.INSTANCE.setBallSlipFactor(SLIP_FACTOR[1])),
                new FollowPath(paths.firstSpikeIntake),
                new ParallelGroup(
                    RobotNF.INSTANCE.delayedIdle(0.66),
                    cruiseShootBalls(
                            new double[] {SHOOT_TIME, 0.35},
                            0.833,
                            paths.firstSpikeReturn,
                            new PowerAdjustedPath(paths.firstSpikeReturn, 1, 10, 0.8, 1),
                            new SequentialGroup(
                                    ShooterNF.INSTANCE.setVel(paths.secondSpikeReturn.endPose(), 25),
                                    new InstantCommand(() -> ShooterNF.INSTANCE.setBallSlipFactor(SLIP_FACTOR[2])),
                                    new FollowPath(paths.secondSpikeIntake)
                            )
                    )
                ),

                new ParallelGroup(
                        RobotNF.INSTANCE.delayedIdle(0.66),
                        cruiseShootBalls(
                                new double[] {SHOOT_TIME, 0.35},
                                0.88,
                                paths.secondSpikeReturn,
                                new PowerAdjustedPath(paths.secondSpikeReturn, 1, 5, 0.92, 1),
                                new SequentialGroup(
                                        ShooterNF.INSTANCE.setVel(paths.firstGateReturn.endPose()),
                                        new InstantCommand(() -> ShooterNF.INSTANCE.setBallSlipFactor(SLIP_FACTOR[3])),
                                        gate(paths.firstGateIntake)
                                )
                        )
                ),

                new ParallelGroup(
                        RobotNF.INSTANCE.delayedIdle(0.66),
                        cruiseShootBalls(
                                new double[] {SHOOT_TIME, 0.35},
                                0.92,
                                paths.firstGateReturn,
                                new PowerAdjustedPath(paths.firstGateReturn, 1, 5, 0.93, 1),
                                new SequentialGroup(
                                        ShooterNF.INSTANCE.setVel(paths.secondGateReturn.endPose()),
                                        new InstantCommand(() -> ShooterNF.INSTANCE.setBallSlipFactor(SLIP_FACTOR[4])),
                                        gate(paths.secondGateIntake)
                                )
                        )
                ),

                new ParallelGroup(
                        RobotNF.INSTANCE.delayedIdle(0.66),
                        cruiseShootBalls(
                                new double[] {SHOOT_TIME, 0.35},
                                0.92,
                                paths.secondGateReturn,
                                new PowerAdjustedPath(paths.secondGateReturn, 1, 5, 0.93, 1),
                                new SequentialGroup(
                                        ShooterNF.INSTANCE.setVel(paths.thirdGateReturn.endPose()),
                                        new InstantCommand(() -> ShooterNF.INSTANCE.setBallSlipFactor(SLIP_FACTOR[5])),
                                        gate(paths.thirdGateIntake)
                                )
                        )
                ),

                new ParallelGroup(
                        RobotNF.INSTANCE.delayedIdle(0.66),
                        cruiseShootBalls(
                                new double[] {SHOOT_TIME, 0.35},
                                0.833,
                                paths.thirdGateReturn,
                                new PowerAdjustedPath(paths.thirdGateReturn, 1, 5, 0.93, 1),
                                new SequentialGroup(
                                        ShooterNF.INSTANCE.setVel(paths.thirdSpikeReturn.endPose(), 15),
                                        new InstantCommand(() -> ShooterNF.INSTANCE.setBallSlipFactor(SLIP_FACTOR[6])),
                                        new FollowPath(paths.thirdSpikeIntake)
                                )
                        )
                ),

                new ParallelGroup(
                        RobotNF.INSTANCE.delayedIdle(0.6),
                        new PowerAdjustedPath(paths.thirdSpikeReturn, 1, 5, 0.88, 1),
                        RobotNF.INSTANCE.shootBalls(0.75, 8, paths.thirdSpikeReturn)
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
                                new Delay(0.875),
                                new InstantCommand(() -> PedroComponent.follower().setMaxPower(0.665))
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
