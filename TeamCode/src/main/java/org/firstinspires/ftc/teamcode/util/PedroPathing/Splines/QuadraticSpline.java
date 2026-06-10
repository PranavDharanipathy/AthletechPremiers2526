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

    /**
     * Interpolating quadratic spline:
     * Passes through p0, p1, p2 at t = 0, 0.5, 1
     */
    @Override
    public Pose getPose(double t) {
        t = Math.max(0.0, Math.min(1.0, t));

        Pose p0 = getControlPoints().get(0);
        Pose p1 = getControlPoints().get(1);
        Pose p2 = getControlPoints().get(2);

        // Basis for interpolation through 3 points at t = 0, 0.5, 1
        double a = 2 * (t - 0.5) * (t - 1.0);
        double b = -4 * t * (t - 1.0);
        double c = 2 * t * (t - 0.5);

        double x = a * p0.getX() + b * p1.getX() + c * p2.getX();
        double y = a * p0.getY() + b * p1.getY() + c * p2.getY();

        return new Pose(x, y);
    }

    @Override
    public Vector getDerivative(double t) {
        t = Math.max(0.0, Math.min(1.0, t));

        Pose p0 = getControlPoints().get(0);
        Pose p1 = getControlPoints().get(1);
        Pose p2 = getControlPoints().get(2);

        double dx =
                (4 * t - 3) * p0.getX()
                        + (-8 * t + 4) * p1.getX()
                        + (4 * t - 1) * p2.getX();

        double dy =
                (4 * t - 3) * p0.getY()
                        + (-8 * t + 4) * p1.getY()
                        + (4 * t - 1) * p2.getY();

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
                4 * p0.getX()
                        - 8 * p1.getX()
                        + 4 * p2.getX();

        double ddy =
                4 * p0.getY()
                        - 8 * p1.getY()
                        + 4 * p2.getY();

        Vector out = new Vector();
        out.setOrthogonalComponents(ddx, ddy);
        return out;
    }
}