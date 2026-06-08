package org.firstinspires.ftc.teamcode.util.PedroPathing;

import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.commands.Command;
import dev.nextftc.extensions.pedro.PedroComponent;

public class PowerAdjustedPath extends Command {

    private final PathChain path;
    private final double initialMaxPower, distanceRemaining, maxPowerAfterDistance, endMaxPower;

    public PowerAdjustedPath(PathChain path, double initialMaxPower, double distanceRemaining, double maxPowerAfterDistance, double endMaxPower) {

        this.path = path;
        this.initialMaxPower = initialMaxPower;
        this.distanceRemaining = distanceRemaining;
        this.maxPowerAfterDistance = maxPowerAfterDistance;
        this.endMaxPower = endMaxPower;
    }

    //states
    private boolean isComplete = false;

    private boolean distanceCrossed = false;

    private boolean pathComplete = false;

    @Override
    public boolean isDone() {
        return isComplete;
    }//!PedroComponent.Companion.follower().isBusy()

    @Override
    public void update() {

        if (!distanceCrossed && PedroComponent.Companion.follower().getTotalDistanceRemaining() < distanceRemaining) {
            distanceCrossed = true;
            PedroComponent.Companion.follower().setMaxPower(maxPowerAfterDistance);
        }

        if (!pathComplete && !PedroComponent.Companion.follower().isBusy()) {
            pathComplete = true;
            PedroComponent.Companion.follower().setMaxPower(endMaxPower);
            isComplete = true;
        }
    }

    @Override
    public void start() {

        PedroComponent.Companion.follower().setMaxPower(initialMaxPower);
        PedroComponent.Companion.follower().followPath(path, true);
    }

    @Override
    public void stop(boolean interrupted) {
        if (interrupted) {
            PedroComponent.Companion.follower().breakFollowing();
        }
    }
}
