package org.firstinspires.ftc.teamcode.util.PedroPathing.Splines;

import com.pedropathing.geometry.CustomCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;

import java.util.List;

public abstract class PolynomialSpline extends CustomCurve {

    public PolynomialSpline(List<Pose> points, PathConstraints constraints) {
        super(points, constraints);
    }

    protected double clamp(double t) {
        return Math.max(0.0, Math.min(1.0, t));
    }
}