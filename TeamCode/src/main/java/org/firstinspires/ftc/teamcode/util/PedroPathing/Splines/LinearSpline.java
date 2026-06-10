package org.firstinspires.ftc.teamcode.util.PedroPathing.Splines;

import com.pedropathing.geometry.CustomCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathConstraints;

import java.util.Arrays;

public class LinearSpline extends CustomCurve {

    public LinearSpline(Pose p0, Pose p1) {
        super(p0, p1);
    }

    public LinearSpline(
            Pose p0,
            Pose p1,
            PathConstraints constraints
    ) {
        super(Arrays.asList(p0, p1), constraints);
    }

    @Override
    public String pathType() {
        return "Linear Spline";
    }

    @Override
    public LinearSpline getReversed() {
        return new LinearSpline(
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

        double x =
                (1 - t) * p0.getX()
                        + t * p1.getX();

        double y =
                (1 - t) * p0.getY()
                        + t * p1.getY();

        return new Pose(x, y);
    }

    @Override
    public Vector getDerivative(double t) {

        Pose p0 = getControlPoints().get(0);
        Pose p1 = getControlPoints().get(1);

        Vector out = new Vector();
        out.setOrthogonalComponents(
                p1.getX() - p0.getX(),
                p1.getY() - p0.getY()
        );

        return out;
    }

    @Override
    public Vector getSecondDerivative(double t) {
        return new Vector(0, 0);
    }
}