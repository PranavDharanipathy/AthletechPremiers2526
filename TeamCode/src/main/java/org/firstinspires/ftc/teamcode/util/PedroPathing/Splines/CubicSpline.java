package org.firstinspires.ftc.teamcode.util.PedroPathing.Splines;

import com.pedropathing.geometry.CustomCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathConstraints;

import java.util.Arrays;

public class CubicSpline extends CustomCurve {

    public CubicSpline(
            Pose p0,
            Pose p1,
            Pose p2,
            Pose p3
    ) {
        super(p0, p1, p2, p3);
    }

    public CubicSpline(
            Pose p0,
            Pose p1,
            Pose p2,
            Pose p3,
            PathConstraints constraints
    ) {
        super(Arrays.asList(p0, p1, p2, p3), constraints);
    }

    @Override
    public String pathType() {
        return "Cubic Spline";
    }

    @Override
    public CubicSpline getReversed() {
        return new CubicSpline(
                getControlPoints().get(3),
                getControlPoints().get(2),
                getControlPoints().get(1),
                getControlPoints().get(0),
                getPathConstraints()
        );
    }

    @Override
    public Pose getPose(double t) {
        t = Math.max(0.0, Math.min(1.0, t));

        Pose p0 = getControlPoints().get(0);
        Pose p1 = getControlPoints().get(1);
        Pose p2 = getControlPoints().get(2);
        Pose p3 = getControlPoints().get(3);

        double u = 1.0 - t;

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
        t = Math.max(0.0, Math.min(1.0, t));

        Pose p0 = getControlPoints().get(0);
        Pose p1 = getControlPoints().get(1);
        Pose p2 = getControlPoints().get(2);
        Pose p3 = getControlPoints().get(3);

        double u = 1.0 - t;

        double dx =
                3*u*u*(p1.getX()-p0.getX())
                        + 6*u*t*(p2.getX()-p1.getX())
                        + 3*t*t*(p3.getX()-p2.getX());

        double dy =
                3*u*u*(p1.getY()-p0.getY())
                        + 6*u*t*(p2.getY()-p1.getY())
                        + 3*t*t*(p3.getY()-p2.getY());

        Vector out = new Vector();
        out.setOrthogonalComponents(dx, dy);
        return out;
    }

    @Override
    public Vector getSecondDerivative(double t) {
        t = Math.max(0.0, Math.min(1.0, t));

        Pose p0 = getControlPoints().get(0);
        Pose p1 = getControlPoints().get(1);
        Pose p2 = getControlPoints().get(2);
        Pose p3 = getControlPoints().get(3);

        double u = 1.0 - t;

        double ddx =
                6*u*(p2.getX()-2*p1.getX()+p0.getX())
                        + 6*t*(p3.getX()-2*p2.getX()+p1.getX());

        double ddy =
                6*u*(p2.getY()-2*p1.getY()+p0.getY())
                        + 6*t*(p3.getY()-2*p2.getY()+p1.getY());

        Vector out = new Vector();
        out.setOrthogonalComponents(ddx, ddy);
        return out;
    }
}