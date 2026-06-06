package org.firstinspires.ftc.teamcode.util.PedroPathing;

import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.commands.Command;
import dev.nextftc.extensions.pedro.PedroComponent;

public class CancelableFollowPath extends Command {

    private final PathChain path;
    private final boolean holdEnd;
    private final double cancelTime;

    private final ElapsedTime timer;

    /// @param cancelTime in seconds
    public CancelableFollowPath(PathChain path, double cancelTime) {
        this(path, false, cancelTime);
    }

    /// @param cancelTime in seconds
    public CancelableFollowPath(PathChain path, boolean holdEnd, double cancelTime) {

        this.path = path;
        this.holdEnd = holdEnd;
        this.cancelTime = cancelTime;

        timer = new ElapsedTime();
    }

    @Override
    public boolean isDone() {

        return !PedroComponent.Companion.follower().isBusy() || timer.seconds() > cancelTime;

    }

    @Override
    public void start() {
        PedroComponent.Companion.follower().followPath(path, holdEnd);
        timer.reset();
    }

    @Override
    public void stop(boolean interrupted) {
        if (interrupted) {
            PedroComponent.Companion.follower().breakFollowing();
        }
    }
}
