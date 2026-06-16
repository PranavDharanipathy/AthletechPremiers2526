package org.firstinspires.ftc.teamcode.Auto.Blue;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Constants.Calculations;
import org.firstinspires.ftc.teamcode.util.PedroPathing.MirroredPose;

@Config
public class BlueAllianceQualsPaths {

    public static double[] GSX = {2, 2.1, 2.1, 2.2};
    public static double[] GSY = {1.5, 1.6, 1.7, 1.8};
    public static double[] GX = {3, 3, 3, 3};
    public static double[] GY = {3, 3, 4, 4.5};
    public static double[] GHEADING_DEG = {17, 17, 17, 17};

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

    public BlueAllianceQualsPaths(Follower follower, Pose startPose) {

        preload = follower.pathBuilder().addPath(
                        new BezierLine(
                                startPose,
                                new MirroredPose(90, 80.828)
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0, 0.1,
                                        HeadingInterpolator.constant(startPose.getHeading())
                                ),

                                new HeadingInterpolator.PiecewiseNode(
                                        0.1, 0.7,
                                        HeadingInterpolator.tangent.reverse()
                                ),

                                new HeadingInterpolator.PiecewiseNode(
                                        0.7, 1,
                                        HeadingInterpolator.constant(Calculations.mirrorHeading(0))
                                )
                        )
                )
                .build();

        firstSpikeIntake = follower.pathBuilder().addPath(
                        new BezierLine(
                                new MirroredPose(85.581, 80.828),
                                new MirroredPose(125, 80.828)
                        )
                )
                .setTangentHeadingInterpolation()
                //.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        firstSpikeReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new MirroredPose(125, 80.828),
                                new MirroredPose(96, 83)
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0, 0.33,
                                        HeadingInterpolator.constant(Calculations.mirrorHeading(0))
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.33, 1,
                                        HeadingInterpolator.linear(Calculations.mirrorHeading(0), Calculations.mirrorHeading(20), 0.9)
                                )
                        )
                )
                .build();

        secondSpikeIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new MirroredPose(96, 83),
                                new MirroredPose(111, 59.858),
                                new MirroredPose(120, 58)
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0, 0.8,
                                        HeadingInterpolator.tangent
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.8, 1,
                                        HeadingInterpolator.constant(Calculations.mirrorHeading(Math.toRadians(-14)))
                                )
                        )
                )
                .build();

        secondSpikeReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new MirroredPose(120, 58),
                                new MirroredPose(95, 83)
                        )
                )
                .setLinearHeadingInterpolation(Calculations.mirrorHeading(Math.toRadians(-14)), Calculations.mirrorHeading(Math.toRadians(343)))
                .build();

        firstGateIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new MirroredPose(95, 83),
                                new MirroredPose(101.934+GSX[0], 57.3+GSY[0]),
                                new MirroredPose(126.943+GX[0], 56.790+GY[0])
                        )
                )
                .setLinearHeadingInterpolation(Calculations.mirrorHeading(Math.toRadians(343)), Calculations.mirrorHeading(Math.toRadians(GHEADING_DEG[0])))
                .build();

        firstGateReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new MirroredPose(126.943+GX[0], 56.790+GY[0]),
                                new MirroredPose(96, 86)
                        )
                )
                .setLinearHeadingInterpolation(Calculations.mirrorHeading(Math.toRadians(GHEADING_DEG[0])), Calculations.mirrorHeading(Math.toRadians(343)))
                .build();

        secondGateIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new MirroredPose(96, 86),
                                new MirroredPose(101.934+GSX[1], 57.3+GSY[1]),
                                new MirroredPose(126.943+GX[1], 56.790+GY[1])
                        )
                )
                .setLinearHeadingInterpolation(Calculations.mirrorHeading(Math.toRadians(343)), Calculations.mirrorHeading(Math.toRadians(GHEADING_DEG[1])))
                .build();

        secondGateReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new MirroredPose(126.943+GX[1], 56.790+GY[1]),
                                new MirroredPose(96, 86)
                        )
                )
                .setLinearHeadingInterpolation(Calculations.mirrorHeading(Math.toRadians(GHEADING_DEG[1])), Calculations.mirrorHeading(Math.toRadians(343)))
                .build();

        thirdGateIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new MirroredPose(96, 86),
                                new MirroredPose(101.934+GSX[2], 57.3+GSY[2]),
                                new MirroredPose(126.943+GX[2], 56.790+GY[2])
                        )
                )
                .setLinearHeadingInterpolation(Calculations.mirrorHeading(Math.toRadians(343)), Calculations.mirrorHeading(Math.toRadians(GHEADING_DEG[2])))
                .build();

        thirdGateReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new MirroredPose(126.943+GX[2], 56.790+GY[2]),
                                new MirroredPose(96, 86)
                        )
                )
                .setLinearHeadingInterpolation(Calculations.mirrorHeading(Math.toRadians(GHEADING_DEG[2])), Calculations.mirrorHeading(Math.toRadians(343)))
                .build();

        fourthGateIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new MirroredPose(96, 86),
                                new MirroredPose(101.934+GSX[3], 57.3+GSY[3]),
                                new MirroredPose(126.943+GX[3], 56.790+GY[3])
                        )
                )
                .setLinearHeadingInterpolation(Calculations.mirrorHeading(Math.toRadians(343)), Calculations.mirrorHeading(Math.toRadians(GHEADING_DEG[3])))
                .build();

        fourthGateReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new MirroredPose(126.943+GX[3], 56.790+GY[3]),
                                new MirroredPose(96, 110)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }
}