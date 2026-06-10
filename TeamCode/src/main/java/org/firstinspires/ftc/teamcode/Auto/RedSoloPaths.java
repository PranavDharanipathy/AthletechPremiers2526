package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Constants.Calculations;

@Config
public class RedSoloPaths {

    public static double[] GSX = {0, 0.5, 1};
    public static double[] GSY = {1.5, 1.85, 1.75};
    public static double[] GX = {4, 5, 5};
    public static double[] GY = {-1.5, -0.4, 0};
    public static double[] GHEADING_DEG = {17, 17, 16};

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
                        new Pose(115, 81.828)
                )
        )
        .setTangentHeadingInterpolation()
        //.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
        .build();

        firstSpikeReturn = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(115, 81.828),
                        new Pose(84.890, 82.090)
                )
        )
        .setHeadingInterpolation(
                HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(
                                0, 0.33,
                                HeadingInterpolator.constant(Math.toRadians(0))
                        ),
                        new HeadingInterpolator.PiecewiseNode(
                                0.33, 1,
                                HeadingInterpolator.linear(Math.toRadians(0), Math.toRadians(20), 0.9)
                        )
                )
        )
        .build();

        secondSpikeIntake = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(84.890, 82.090),
                        new Pose(96.620, 59.858),
                        new Pose(112, 58.620)
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
                                HeadingInterpolator.constant(Math.toRadians(-15))
                        )
                )
        )
        //.setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(300))
        .build();

        final BezierLine secondSpikeLine = new BezierLine(
                new Pose(112, 58.620),
                new Pose(85, 80)
        );
        secondSpikeReturn = follower.pathBuilder().addPath(secondSpikeLine)
        .setHeadingInterpolation(
                HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(
                                0, 0.5,
                                HeadingInterpolator.tangent.reverse()
                        ),
                        new HeadingInterpolator.PiecewiseNode(
                                0.5, 1,
                                HeadingInterpolator.linear(Calculations.getTangentAngle(secondSpikeLine), Math.toRadians(343))
                        )
                )
        )
        .build();

        firstGateIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(85, 80),
                                new Pose(101.934+GSX[0], 57.3+GSY[0]),
                                new Pose(126.943+GX[0], 56.790+GY[0])
                        )
        )
        .setLinearHeadingInterpolation(Math.toRadians(343), Math.toRadians(GHEADING_DEG[0]))
        .build();

        firstGateReturn = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(126.943+GX[0], 56.790+GY[0]),
                        new Pose(84.852, 73)
                )
        )
        .setLinearHeadingInterpolation(Math.toRadians(GHEADING_DEG[0]), Math.toRadians(343))
        .build();

        secondGateIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(84.852, 73),
                                new Pose(101.934+GSX[1], 57.3+GSY[1]),
                                new Pose(126.943+GX[1], 56.790+GY[1])
                        )
        )
        .setLinearHeadingInterpolation(Math.toRadians(343), Math.toRadians(GHEADING_DEG[1]))
        .build();

        secondGateReturn = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(126.943+GX[1], 56.790+GY[1]),
                        new Pose(84.852, 73)
                )
        )
        .setLinearHeadingInterpolation(Math.toRadians(GHEADING_DEG[1]), Math.toRadians(343))
        .build();

        thirdGateIntake = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(84.852, 73),
                        new Pose(101.934+GSX[2], 57.3+GSY[2]),
                        new Pose(126.943+GX[2], 56.790+GY[2])
                )
        )
        .setLinearHeadingInterpolation(Math.toRadians(343), Math.toRadians(GHEADING_DEG[2]))
        .build();

        thirdGateReturn = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(126.943+GX[2], 56.790+GY[2]),
                        new Pose(84.852, 73)
                )
        )
        .setLinearHeadingInterpolation(Math.toRadians(GHEADING_DEG[2]), Math.toRadians(343))
        .build();

        thirdSpikeIntake = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(84.852, 73),
                        new Pose(99.811, 35.854),
                        new Pose(120.853, 32.854)
                )
        )
        .setTangentHeadingInterpolation()
        .build();

        thirdSpikeReturn = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(120.853, 34.854),
                        new Pose(89, 105)
                )
        )
            .setHeadingInterpolation(
                    HeadingInterpolator.piecewise(
                            new HeadingInterpolator.PiecewiseNode(
                                    0, 0.8,
                                    HeadingInterpolator.tangent.reverse()
                            ),
                            new HeadingInterpolator.PiecewiseNode(
                                    0.8, 1,
                                    HeadingInterpolator.linear(Calculations.getTangentAngle(secondSpikeLine), Math.toRadians(-30))
                            )
                    )
            )
        .build();
    }
}