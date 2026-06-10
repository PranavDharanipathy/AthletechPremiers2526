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

    private Pose catmullRom(
            Pose p0,
            Pose p1,
            Pose p2,
            Pose p3,
            double t
    ) {
        double t2 = t * t;
        double t3 = t2 * t;

        double x =
                0.5 * (
                        2 * p1.getX()
                                + (-p0.getX() + p2.getX()) * t
                                + (2 * p0.getX()
                                - 5 * p1.getX()
                                + 4 * p2.getX()
                                - p3.getX()) * t2
                                + (-p0.getX()
                                + 3 * p1.getX()
                                - 3 * p2.getX()
                                + p3.getX()) * t3
                );

        double y =
                0.5 * (
                        2 * p1.getY()
                                + (-p0.getY() + p2.getY()) * t
                                + (2 * p0.getY()
                                - 5 * p1.getY()
                                + 4 * p2.getY()
                                - p3.getY()) * t2
                                + (-p0.getY()
                                + 3 * p1.getY()
                                - 3 * p2.getY()
                                + p3.getY()) * t3
                );

        return new Pose(x, y);
    }

    @Override
    public Pose getPose(double t) {
        t = Math.max(0.0, Math.min(1.0, t));

        Pose p0 = getControlPoints().get(0);
        Pose p1 = getControlPoints().get(1);
        Pose p2 = getControlPoints().get(2);
        Pose p3 = getControlPoints().get(3);

        if (t <= 1.0 / 3.0) {
            double localT = t * 3.0;
            return catmullRom(p0, p0, p1, p2, localT);
        }

        if (t <= 2.0 / 3.0) {
            double localT = (t - 1.0 / 3.0) * 3.0;
            return catmullRom(p0, p1, p2, p3, localT);
        }

        double localT = (t - 2.0 / 3.0) * 3.0;
        return catmullRom(p1, p2, p3, p3, localT);
    }

    @Override
    public Vector getDerivative(double t) {
        double dt = 1e-5;

        Pose a = getPose(Math.max(0, t - dt));
        Pose b = getPose(Math.min(1, t + dt));

        Vector out = new Vector();
        out.setOrthogonalComponents(
                (b.getX() - a.getX()) / (2 * dt),
                (b.getY() - a.getY()) / (2 * dt)
        );

        return out;
    }

    @Override
    public Vector getSecondDerivative(double t) {
        double dt = 1e-4;

        Vector d1 = getDerivative(Math.max(0, t - dt));
        Vector d2 = getDerivative(Math.min(1, t + dt));

        Vector out = new Vector();
        out.setOrthogonalComponents(
                (d2.getXComponent() - d1.getXComponent()) / (2 * dt),
                (d2.getYComponent() - d1.getYComponent()) / (2 * dt)
        );

        return out;
    }
}