package org.firstinspires.ftc.teamcode.Auto;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

@Config
public class RedFarPaths {

    public PathChain mainChain;

    public RedFarPaths(Follower follower, Pose startPose) {

        mainChain = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                startPose,
                                new Pose(120.000, 6)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(120.000, 6),
                                new Pose(94, 8)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addPath(
                        new BezierLine(
                                new Pose(94, 8),
                                new Pose(120.000, 8)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(
                                new Pose(120.000, 8),
                                new Pose(94, 8)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addPath(
                        new BezierCurve(
                                new Pose(94, 8),
                                new Pose(104.983, 13.410),
                                new Pose(114.084, 22.336)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(
                                new Pose(114.084, 22.336),
                                new Pose(90, 12)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addPath(
                        new BezierCurve(
                                new Pose(90, 12),
                                new Pose(104.983, 13.410),
                                new Pose(114.084, 22.336)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(
                                new Pose(114.084, 22.336),
                                new Pose(90, 12)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addPath(
                        new BezierCurve(
                                new Pose(90, 12),
                                new Pose(104.983, 13.410),
                                new Pose(114.084, 22.336)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(
                                new Pose(114.084, 22.336),
                                new Pose(90, 12)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addPath(
                        new BezierCurve(
                                new Pose(90, 12),
                                new Pose(104.983, 13.410),
                                new Pose(114.084, 22.336)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(
                                new Pose(114.084, 22.336),
                                new Pose(90, 12)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }
}