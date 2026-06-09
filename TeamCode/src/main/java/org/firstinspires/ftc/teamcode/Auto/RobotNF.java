package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Auto.autosubsystems.ShooterNF;
import org.firstinspires.ftc.teamcode.Auto.autosubsystems.IntakeNF;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.SubsystemGroup;
import dev.nextftc.extensions.pedro.PedroComponent;

public class RobotNF extends SubsystemGroup {


    private RobotNF() {

        super(
                IntakeNF.INSTANCE,
                ShooterNF.INSTANCE
        );
    }

    public static final RobotNF INSTANCE = new RobotNF();

    public final Command delayedIdle(double delay) {

        return new SequentialGroup(
                new Delay(delay),
                IntakeNF.INSTANCE.idle()
        );
    }

    public final Command shootBalls(double shootTime) {

        return new SequentialGroup(

                IntakeNF.INSTANCE.intake(),
                IntakeNF.INSTANCE.blocker(true),
                new Delay(shootTime),
                IntakeNF.INSTANCE.blocker(false)
        );
    }

    public final Command shootBalls(double shootTime, PathChain pathChain) {

        return new SequentialGroup(

                new WaitUntil(() -> pathChain.lastPath().isAtParametricEnd()),

                IntakeNF.INSTANCE.intake(),
                IntakeNF.INSTANCE.blocker(true),
                new Delay(shootTime),
                IntakeNF.INSTANCE.blocker(false)
        );
    }

    public final Command shootBalls(double shootTime, double distance, PathChain pathChain) {

        return new SequentialGroup(

                new WaitUntil(() -> pathChain.lastPath().getDistanceRemaining() <= distance),

                IntakeNF.INSTANCE.intake(),
                IntakeNF.INSTANCE.blocker(true),
                new Delay(shootTime),
                IntakeNF.INSTANCE.blocker(false)
        );
    }

    public final Command cruiseShootBalls(double shootTime, double cruiseShootPower, PathChain pathChain) {

        return new SequentialGroup(

                new WaitUntil(() -> pathChain.lastPath().isAtParametricEnd()),
                new InstantCommand(() -> PedroComponent.follower().setMaxPower(cruiseShootPower)),

                IntakeNF.INSTANCE.intake(),
                IntakeNF.INSTANCE.blocker(true),
                new Delay(shootTime),
                IntakeNF.INSTANCE.blocker(false),
                new InstantCommand(() -> PedroComponent.follower().setMaxPower(1))
        );
    }

    public final Command cruiseShootBalls(double shootTime, double cruiseShootPower, double distanceRemaining, PathChain pathChain) {

        return new SequentialGroup(

                new WaitUntil(() -> pathChain.lastPath().getDistanceRemaining() <= distanceRemaining),
                new InstantCommand(() -> PedroComponent.follower().setMaxPower(cruiseShootPower)),

                IntakeNF.INSTANCE.intake(),
                IntakeNF.INSTANCE.blocker(true),
                new Delay(shootTime),
                IntakeNF.INSTANCE.blocker(false),
                new InstantCommand(() -> PedroComponent.follower().setMaxPower(1))
        );
    }

    public final void end() {
        IntakeNF.INSTANCE.end();
        ShooterNF.INSTANCE.end();
    }
}