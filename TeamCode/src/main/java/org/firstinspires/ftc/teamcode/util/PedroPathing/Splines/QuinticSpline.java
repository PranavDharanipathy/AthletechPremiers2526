package org.firstinspires.ftc.teamcode.util.PedroPathing.Splines;

import com.pedropathing.geometry.CustomCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathConstraints;

import java.util.Arrays;

public class QuinticSpline extends CustomCurve {

    public QuinticSpline(
            Pose p0,
            Pose p1,
            Pose p2,
            Pose p3,
            Pose p4,
            Pose p5
    ) {
        super(p0, p1, p2, p3, p4, p5);
    }

    public QuinticSpline(
            Pose p0,
            Pose p1,
            Pose p2,
            Pose p3,
            Pose p4,
            Pose p5,
            PathConstraints constraints
    ) {
        super(Arrays.asList(
                p0, p1, p2, p3, p4, p5
        ), constraints);
    }

    @Override
    public String pathType() {
        return "Quintic Spline";
    }

    @Override
    public QuinticSpline getReversed() {
        return new QuinticSpline(
                getControlPoints().get(5),
                getControlPoints().get(4),
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
        Pose p4 = getControlPoints().get(4);
        Pose p5 = getControlPoints().get(5);

        double u = 1.0 - t;

        double x =
                Math.pow(u, 5) * p0.getX()
                        + 5 * Math.pow(u, 4) * t * p1.getX()
                        + 10 * Math.pow(u, 3) * t * t * p2.getX()
                        + 10 * u * u * t * t * t * p3.getX()
                        + 5 * u * t * t * t * t * p4.getX()
                        + Math.pow(t, 5) * p5.getX();

        double y =
                Math.pow(u, 5) * p0.getY()
                        + 5 * Math.pow(u, 4) * t * p1.getY()
                        + 10 * Math.pow(u, 3) * t * t * p2.getY()
                        + 10 * u * u * t * t * t * p3.getY()
                        + 5 * u * t * t * t * t * p4.getY()
                        + Math.pow(t, 5) * p5.getY();

        return new Pose(x, y);
    }

    @Override
    public Vector getDerivative(double t) {
        t = Math.max(0.0, Math.min(1.0, t));

        Pose p0 = getControlPoints().get(0);
        Pose p1 = getControlPoints().get(1);
        Pose p2 = getControlPoints().get(2);
        Pose p3 = getControlPoints().get(3);
        Pose p4 = getControlPoints().get(4);
        Pose p5 = getControlPoints().get(5);

        double u = 1.0 - t;

        double dx =
                5*Math.pow(u,4)*(p1.getX()-p0.getX())
                        + 20*Math.pow(u,3)*t*(p2.getX()-p1.getX())
                        + 30*u*u*t*t*(p3.getX()-p2.getX())
                        + 20*u*t*t*t*(p4.getX()-p3.getX())
                        + 5*t*t*t*t*(p5.getX()-p4.getX());

        double dy =
                5*Math.pow(u,4)*(p1.getY()-p0.getY())
                        + 20*Math.pow(u,3)*t*(p2.getY()-p1.getY())
                        + 30*u*u*t*t*(p3.getY()-p2.getY())
                        + 20*u*t*t*t*(p4.getY()-p3.getY())
                        + 5*t*t*t*t*(p5.getY()-p4.getY());

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
        Pose p4 = getControlPoints().get(4);
        Pose p5 = getControlPoints().get(5);

        double u = 1.0 - t;

        double ddx =
                20*Math.pow(u,3)*(p2.getX()-2*p1.getX()+p0.getX())
                        + 60*u*u*t*(p3.getX()-2*p2.getX()+p1.getX())
                        + 60*u*t*t*(p4.getX()-2*p3.getX()+p2.getX())
                        + 20*t*t*t*(p5.getX()-2*p4.getX()+p3.getX());

        double ddy =
                20*Math.pow(u,3)*(p2.getY()-2*p1.getY()+p0.getY())
                        + 60*u*u*t*(p3.getY()-2*p2.getY()+p1.getY())
                        + 60*u*t*t*(p4.getY()-2*p3.getY()+p2.getY())
                        + 20*t*t*t*(p5.getY()-2*p4.getY()+p3.getY());

        Vector out = new Vector();
        out.setOrthogonalComponents(ddx, ddy);
        return out;
    }
}