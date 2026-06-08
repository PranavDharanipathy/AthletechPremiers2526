package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

@Config
public class RedSoloPaths {

    public static double GSX = 0;
    public static double GSY = 1.5;
    public static double GX = 0;
    public static double GY = -0.9;
    public static double GHEADING_DEG = 25;

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

        preload = follower.pathBuilder().addPath(
                new BezierLine(
                        startPose,
                        new Pose(85.581, 82.220)
                )
        )
        .setHeadingInterpolation(
                HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(
                                0, 0.1,
                                HeadingInterpolator.constant(startPose.getHeading())
                        ),

                        new HeadingInterpolator.PiecewiseNode(
                                0.1, 0.3,
                                HeadingInterpolator.linear(Math.toRadians(startPose.getHeading()), Math.toRadians(0))
                        ),

                        new HeadingInterpolator.PiecewiseNode(
                                0.3, 1,
                                HeadingInterpolator.constant(Math.toRadians(0))
                        )
                )
        )
        .build();

        firstSpikeIntake = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(85.581, 82.220),
                        new Pose(109, 81.828)
                )
        )
        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
        .build();

        firstSpikeReturn = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(109, 81.828),
                        new Pose(88.451, 82.028)
                )
        )
        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
        .build();

        secondSpikeIntake = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(88.451, 82.028),
                        new Pose(117.188, 60.235)
                )
        )
        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(300))
        .build();

        secondSpikeReturn = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(117.188, 60.235),
                        new Pose(88.896, 82.091)
                )
        )
        .setLinearHeadingInterpolation(Math.toRadians(300), Math.toRadians(320))
        .build();

        firstGateIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(88.896, 82.091),
                                new Pose(101.934+GSX, 57.3+GSY),
                                new Pose(126.943+GX, 56.790+GY)
                        )
        )
        .setLinearHeadingInterpolation(Math.toRadians(320), Math.toRadians(GHEADING_DEG))
        .build();

        firstGateReturn = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(126.943+GX, 56.790+GY),
                        new Pose(88.890, 82.090)
                )
        )
        .setLinearHeadingInterpolation(Math.toRadians(GHEADING_DEG), Math.toRadians(320))
        .build();

        secondGateIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(88.890, 82.090),
                                new Pose(101.934+GSX, 57.3+GSY),
                                new Pose(126.943+GX, 56.790+GY)
                        )
        )
        .setLinearHeadingInterpolation(Math.toRadians(320), Math.toRadians(GHEADING_DEG))
        .build();

        secondGateReturn = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(126.943+GX, 56.790+GY),
                        new Pose(88.890, 82.090)
                )
        )
        .setLinearHeadingInterpolation(Math.toRadians(GHEADING_DEG), Math.toRadians(320))
        .build();

        thirdGateIntake = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(88.890, 82.090),
                        new Pose(101.934+GSX, 57.3+GSY),
                        new Pose(126.943+GX, 56.790+GY)
                )
        )
        .setLinearHeadingInterpolation(Math.toRadians(320), Math.toRadians(GHEADING_DEG))
        .build();

        thirdGateReturn = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(126.943+GX, 56.790+GY),
                        new Pose(88.890, 82.090)
                )
        )
        .setLinearHeadingInterpolation(Math.toRadians(GHEADING_DEG), Math.toRadians(320))
        .build();

        thirdSpikeIntake = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(88.890, 82.090),
                        new Pose(99.811, 36.130),
                        new Pose(120.853, 34.854)
                )
        )
        .setTangentHeadingInterpolation()
        .build();

        thirdSpikeReturn = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(120.853, 34.854),
                        new Pose(92.974, 17)
                )
        )
        .setTangentHeadingInterpolation()
        .setReversed()
        .build();

        hpSpikeIntake = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(92.974, 17),
                        new Pose(106.207, 7.871),
                        new Pose(122.080, 7.8)
                )
        )
        .setTangentHeadingInterpolation()
        .build();

        hpSpikeReturn = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(122.080, 7.8),
                        new Pose(86.219, 8.667)
                )
        )
        .setTangentHeadingInterpolation()
        .setReversed()
        .build();
    }
}