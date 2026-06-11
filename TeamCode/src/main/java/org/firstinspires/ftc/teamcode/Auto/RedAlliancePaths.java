package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.util.PedroPathing.Splines.LinearSpline;
import org.firstinspires.ftc.teamcode.util.PedroPathing.Splines.QuadraticSpline;

public class RedAlliancePaths {

    public static double GSHOOTX = 101;
    public static double GSHOOTY = 88;

    public static double[] GSX = {2, 2, 2, 2};
    public static double[] GSY = {1.5, 1.4, 1.4, 1.5};
    public static double[] GX = {6, 6, 6, 6};
    public static double[] GY = {2, 2, 2, 2};
    public static double[] GHEADING_DEG = {16.5, 16.5, 16.5, 16.5};

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
    public PathChain fourthGateIntake;
    public PathChain fourthGateReturn;

    public RedAlliancePaths(Follower follower, Pose startPose) {

        final Pose gateShootPose = new Pose(GSHOOTX, GSHOOTY);

        preload = follower.pathBuilder()
                .addPath(
                        new LinearSpline(
                                startPose,
                                new Pose(108.595, 98.058)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();

        firstSpikeIntake = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(108.595, 98.058),
                                new Pose(128, 85.477),
                                new Pose(116, 81.5),
                                new Pose(125.520, 78.884)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-90), 0.9)
                .build();

        firstSpikeReturn = follower.pathBuilder()
                .addPath(
                        new QuadraticSpline(
                                new Pose(125.520, 78.884),
                                new Pose(113.177, 78.022),
                                new Pose(103.533, 92.381)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(340))
                .build();

        secondSpikeIntake = follower.pathBuilder()
                .addPath(
                        new LinearSpline(
                                new Pose(103.533, 92.381),
                                new Pose(123, 60.804)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(340), Math.toRadians(-90))
                .build();

        secondSpikeReturn = follower.pathBuilder()
                .addPath(
                        new LinearSpline(
                                new Pose(123, 60.804),
                                gateShootPose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(340))
                .build();

        firstGateIntake = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                gateShootPose,
                                new Pose(101.934+GSX[0], 59.800+GSY[0]),
                                new Pose(126.943+GX[0], 56.790+GY[0])
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(340), Math.toRadians(GHEADING_DEG[0]))
                .build();

        firstGateReturn = follower.pathBuilder()
                .addPath(
                        new LinearSpline(
                                new Pose(126.943+GX[0], 56.790+GY[0]),
                                gateShootPose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(GHEADING_DEG[0]), Math.toRadians(340))
                .build();

        secondGateIntake = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                gateShootPose,
                                new Pose(101.934+GSX[1], 59.800+GSY[1]),
                                new Pose(126.943+GX[1], 56.790+GY[1])
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(340), Math.toRadians(GHEADING_DEG[1]))
                .build();

        secondGateReturn = follower.pathBuilder()
                .addPath(
                        new LinearSpline(
                                new Pose(126.943+GX[1], 56.790+GY[1]),
                                gateShootPose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(GHEADING_DEG[1]), Math.toRadians(340))
                .build();

        thirdGateIntake = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                gateShootPose,
                                new Pose(101.934+GSX[2], 59.800+GSY[2]),
                                new Pose(126.943+GX[2], 56.790+GY[2])
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(340), Math.toRadians(GHEADING_DEG[2]))
                .build();

        thirdGateReturn = follower.pathBuilder()
                .addPath(
                        new LinearSpline(
                                new Pose(126.943+GX[2], 56.790+GY[2]),
                                gateShootPose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(GHEADING_DEG[2]), Math.toRadians(340))
                .build();

        fourthGateIntake = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                gateShootPose,
                                new Pose(101.934+GSX[3], 59.800+GSY[3]),
                                new Pose(126.943+GX[3], 56.790+GY[3])
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(340), Math.toRadians(GHEADING_DEG[3]))
                .build();

        fourthGateReturn = follower.pathBuilder()
                .addPath(
                        new LinearSpline(
                                new Pose(126.943+GX[3], 56.790+GY[3]),
                                new Pose(94, 110)
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0, 0.15,
                                        HeadingInterpolator.facingPoint(new Pose(138, 75))
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.15, 1,
                                        HeadingInterpolator.linear(Math.toRadians(GHEADING_DEG[3]), Math.toRadians(-35))
                                )
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(GHEADING_DEG[3]), Math.toRadians(-35))
                .build();

    }
}

