package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auto.autosubsystems.IntakeNF;
import org.firstinspires.ftc.teamcode.Auto.autosubsystems.ShooterNF;
import org.firstinspires.ftc.teamcode.Constants.LocalizationConstants;
import org.firstinspires.ftc.teamcode.Systems.CurrentAlliance;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
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

    public RedSolo() {
        addComponents(
                new PedroComponent(LocalizationConstants::createFollower),
                new SubsystemComponent(
                        RobotNF.INSTANCE,
                        IntakeNF.INSTANCE,
                        ShooterNF.INSTANCE
                ),
                BulkReadComponent.INSTANCE
        );
    }

    private RedSoloPaths paths;

    @Override
    public void onInit() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose startPose = new Pose(53.42, 41.05, Math.toRadians(270)).plus(new Pose(72, 72, 0));
        PedroComponent.follower().setStartingPose(startPose);

        ShooterNF.INSTANCE.provideFollower(PedroComponent.follower());
        ShooterNF.INSTANCE.provideAlliance(ALLIANCE);

        paths = new RedSoloPaths(PedroComponent.follower(), startPose);
    }

    @Override
    public void onWaitForStart() {

        telemetry.addData("turret position", ShooterNF.INSTANCE.turret.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {

        telemetry.clearAll();
        ShooterNF.INSTANCE.start();

        //auto
        auto().schedule();
    }

    @Override
    public void onUpdate() {

    }

    @Override
    public void onStop() {

    }

    private Command auto() {

        return new SequentialGroup(

                new FollowPath(paths.preload),

                new FollowPath(paths.firstSpikeIntake),
                new FollowPath(paths.firstSpikeReturn),

                new FollowPath(paths.secondSpikeIntake),
                new FollowPath(paths.secondSpikeReturn),

                new FollowPath(paths.firstGateIntake),
                new FollowPath(paths.firstGateReturn),

                new FollowPath(paths.secondGateIntake),
                new FollowPath(paths.secondGateReturn),

                new FollowPath(paths.thirdGateIntake),
                new FollowPath(paths.thirdGateReturn),

                new FollowPath(paths.thirdSpikeIntake),
                new FollowPath(paths.thirdSpikeReturn),

                new FollowPath(paths.hpSpikeIntake),
                new FollowPath(paths.hpSpikeReturn)
        );
    }
}
