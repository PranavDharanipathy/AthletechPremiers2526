package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

public class RedSoloPaths {

    private Follower follower;

    public PathChain preload;
    public PathChain firstSpikeIntake;
    public PathChain firstSpikeReturn;
    public PathChain secondSpikeIntake;
    public PathChain secondSpikeReturn;
    public PathChain firstGateIntake;
    public PathChain firstGateReturn;
    public PathChain secondGateIntake;
    public PathChain secondGateReturn;
    public PathChain thirdGateIntake;
    public PathChain thirdGateReturn;
    public PathChain thirdSpikeIntake;
    public PathChain thirdSpikeReturn;
    public PathChain hpSpikeIntake;
    public PathChain hpSpikeReturn;

    public RedSoloPaths(Follower follower, Pose startPose) {

        PathBuilder pb = follower.pathBuilder();

        preload = pb.addPath(
                new BezierLine(
                        startPose,
                        new Pose(85.581, 82.220)
                )
        )
        .setHeadingInterpolation(
                HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(
                                0, 0.1,
                                HeadingInterpolator.constant(Math.toRadians(270))
                        ),

                        new HeadingInterpolator.PiecewiseNode(
                                0.1, 0.3,
                                HeadingInterpolator.linear(Math.toRadians(270), Math.toRadians(0))
                        ),

                        new HeadingInterpolator.PiecewiseNode(
                                0.3, 1,
                                HeadingInterpolator.constant(Math.toRadians(0))
                        )
                )
        )
        .build();

        firstSpikeIntake = pb.addPath(
                new BezierLine(
                        new Pose(85.581, 82.220),
                        new Pose(113, 81.828)
                )
        )
        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
        .build();

        firstSpikeReturn = pb.addPath(
                new BezierLine(
                        new Pose(113, 81.828),
                        new Pose(88.451, 82.028)
                )
        )
        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
        .build();

        secondSpikeIntake = pb.addPath(
                new BezierLine(
                        new Pose(88.451, 82.028),
                        new Pose(117.188, 60.235)
                )
        )
        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(300))
        .build();

        secondSpikeReturn = pb.addPath(
                new BezierLine(
                        new Pose(117.188, 60.235),
                        new Pose(88.896, 82.091)
                )
        )
        .setLinearHeadingInterpolation(Math.toRadians(300), Math.toRadians(320))
        .build();

        firstGateIntake = pb.addPath(
                new BezierLine(
                        new Pose(88.896, 82.091),
                        new Pose(129.983, 60.023)
                )
        )
        .setLinearHeadingInterpolation(Math.toRadians(320), Math.toRadians(30))
        .build();

        firstGateReturn = pb.addPath(
                new BezierLine(
                        new Pose(129.983, 60.023),
                        new Pose(88.890, 82.090)
                )
        )
        .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(320))
        .build();

        secondGateIntake = pb.addPath(
                new BezierLine(
                        new Pose(88.890, 82.090),
                        new Pose(129.980, 60.020)
                )
        )
        .setLinearHeadingInterpolation(Math.toRadians(320), Math.toRadians(30))
        .build();

        secondGateReturn = pb.addPath(
                new BezierLine(
                        new Pose(129.980, 60.020),
                        new Pose(88.890, 82.090)
                )
        )
        .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(320))
        .build();

        thirdGateIntake = pb.addPath(
                new BezierLine(
                        new Pose(88.890, 82.090),
                        new Pose(129.980, 60.020)
                )
        )
        .setLinearHeadingInterpolation(Math.toRadians(320), Math.toRadians(30))
        .build();

        thirdGateReturn = pb.addPath(
                new BezierLine(
                        new Pose(129.980, 60.020),
                        new Pose(88.890, 82.090)
                )
        )
        .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(320))
        .build();

        thirdSpikeIntake = pb.addPath(
                new BezierCurve(
                        new Pose(88.890, 82.090),
                        new Pose(99.811, 36.130),
                        new Pose(120.853, 34.854)
                )
        )
        .setTangentHeadingInterpolation()
        .build();

        thirdSpikeReturn = pb.addPath(
                new BezierLine(
                        new Pose(120.853, 34.854),
                        new Pose(90.974, 12.065)
                )
        )
        .setTangentHeadingInterpolation()
        .setReversed()
        .build();

        hpSpikeIntake = pb.addPath(
                new BezierCurve(
                        new Pose(90.974, 12.065),
                        new Pose(106.207, 7.871),
                        new Pose(129.080, 8.117)
                )
        )
        .setTangentHeadingInterpolation()
        .build();

        hpSpikeReturn = pb.addPath(
                new BezierLine(
                        new Pose(129.080, 8.117),
                        new Pose(86.219, 8.667)
                )
        )
        .setTangentHeadingInterpolation()
        .setReversed()
        .build();
    }
}