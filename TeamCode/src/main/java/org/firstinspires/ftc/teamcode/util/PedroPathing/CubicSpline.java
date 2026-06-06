package org.firstinspires.ftc.teamcode.util.PedroPathing;

import com.pedropathing.geometry.CustomCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathConstraints;

import java.util.Arrays;

public class CubicSpline extends CustomCurve {

    private Pose p0, p1, p2, p3;

    public CubicSpline(Pose p0, Pose p1, Pose p2, Pose p3) {
        super(p0, p1, p2, p3);
    }

    public CubicSpline(Pose p0, Pose p1, Pose p2, Pose p3, PathConstraints constraints) {
        super(Arrays.asList(p0, p1, p2, p3), constraints);
    }

    private void init() {
        p0 = getControlPoints().get(0);
        p1 = getControlPoints().get(1);
        p2 = getControlPoints().get(2);
        p3 = getControlPoints().get(3);
    }

    @Override
    public String pathType() {
        return "Cubic Spline";
    }

    @Override
    public CubicSpline getReversed() {

        CubicSpline spline = new CubicSpline(
                p3, p2, p1, p0,
                getPathConstraints()
        );
        spline.initialize();
        return spline;
    }

    @Override

    public Pose getPose(double t) {
        if (p0 == null) init();

        double u = 1d - t;

        double x =
                u*u*u * p0.getX()
                        + 3*u*u*t * p1.getX()
                        + 3*u*t*t * p2.getX()
                        + t*t*t * p3.getX();

        double y =
                u*u*u * p0.getY()
                        + 3*u*u*t * p1.getY()
                        + 3*u*t*t * p2.getY()
                        + t*t*t * p3.getY();

        return new Pose(x, y);
    }

    @Override
    public Vector getDerivative(double t) {

        if (p0 == null) init();

        double u = 1d - t;

        double dx =
                3*u*u*(p1.getX()-p0.getX())
                        + 6*u*t*(p2.getX()-p1.getX())
                        + 3*t*t*(p3.getX()-p2.getX());

        double dy =
                3*u*u*(p1.getY()-p0.getY())
                        + 6*u*t*(p2.getY()-p1.getY())
                        + 3*t*t*(p3.getY()-p2.getY());

        return new Vector(dx, dy);
    }

    @Override
    public Vector getSecondDerivative(double t) {

        if (p0 == null) init();

        double u = 1d - t;

        double ddx =
                6*u*(p2.getX() - 2*p1.getX() + p0.getX())
                        + 6*t*(p3.getX() - 2*p2.getX() + p1.getX());

        double ddy =
                6*u*(p2.getY() - 2*p1.getY() + p0.getY())
                        + 6*t*(p3.getY() - 2*p2.getY() + p1.getY());

        return new Vector(ddx, ddy);
    }
}