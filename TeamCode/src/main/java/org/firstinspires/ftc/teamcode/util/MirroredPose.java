package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.FuturePose;
import com.pedropathing.geometry.Pose;

public final class MirroredPose implements FuturePose {

    private final Pose pose;
    public MirroredPose(double x, double y, double heading) {
        pose = new Pose(x, y, heading).mirror(144);
    }

    public MirroredPose(double x, double y) {
        this(x, y, 0);
    }

    public MirroredPose() {
        this(0,0,0);
    }

    @Override
    public boolean initialized() {
        return true;
    }

    @Override
    public Pose getPose() {
        return pose;
    }
}
