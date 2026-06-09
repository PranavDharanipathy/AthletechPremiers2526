package org.firstinspires.ftc.teamcode.util.PedroPathing.Splines;

import com.pedropathing.geometry.CustomCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathConstraints;

import java.util.Arrays;

public class QuadraticSpline extends CustomCurve {

    public QuadraticSpline(Pose p0, Pose p1, Pose p2) {
        super(p0, p1, p2);
    }

    public QuadraticSpline(
            Pose p0,
            Pose p1,
            Pose p2,
            PathConstraints constraints
    ) {
        super(Arrays.asList(p0, p1, p2), constraints);
    }

    @Override
    public String pathType() {
        return "Quadratic Spline";
    }

    @Override
    public QuadraticSpline getReversed() {
        return new QuadraticSpline(
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

        double u = 1.0 - t;

        double x =
                u * u * p0.getX()
                        + 2 * u * t * p1.getX()
                        + t * t * p2.getX();

        double y =
                u * u * p0.getY()
                        + 2 * u * t * p1.getY()
                        + t * t * p2.getY();

        return new Pose(x, y);
    }

    @Override
    public Vector getDerivative(double t) {
        t = Math.max(0.0, Math.min(1.0, t));

        Pose p0 = getControlPoints().get(0);
        Pose p1 = getControlPoints().get(1);
        Pose p2 = getControlPoints().get(2);

        double dx =
                2 * (1 - t) * (p1.getX() - p0.getX())
                        + 2 * t * (p2.getX() - p1.getX());

        double dy =
                2 * (1 - t) * (p1.getY() - p0.getY())
                        + 2 * t * (p2.getY() - p1.getY());

        Vector out = new Vector();
        out.setOrthogonalComponents(dx, dy);
        return out;
    }

    @Override
    public Vector getSecondDerivative(double t) {

        Pose p0 = getControlPoints().get(0);
        Pose p1 = getControlPoints().get(1);
        Pose p2 = getControlPoints().get(2);

        double ddx =
                2 * (p2.getX() - 2 * p1.getX() + p0.getX());

        double ddy =
                2 * (p2.getY() - 2 * p1.getY() + p0.getY());

        Vector out = new Vector();
        out.setOrthogonalComponents(ddx, ddy);
        return out;
    }
}