package org.firstinspires.ftc.teamcode.util.PedroPathing;

import com.pedropathing.geometry.CustomCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathConstraints;

import java.util.Arrays;

public class QuinticSpline extends CustomCurve {

    private Pose p0, p1, p2, p3, p4, p5;

    public QuinticSpline( Pose p0, Pose p1, Pose p2, Pose p3, Pose p4, Pose p5) {
        super(p0, p1, p2, p3, p4, p5);
    }

    public QuinticSpline( Pose p0, Pose p1, Pose p2, Pose p3, Pose p4, Pose p5, PathConstraints constraints) {
        super(Arrays.asList(p0, p1, p2, p3, p4, p5), constraints);
    }

    private void init() {
        p0 = getControlPoints().get(0);
        p1 = getControlPoints().get(1);
        p2 = getControlPoints().get(2);
        p3 = getControlPoints().get(3);
        p4 = getControlPoints().get(4);
        p5 = getControlPoints().get(5);
    }

    @Override
    public String pathType() {
        return "Quintic Spline";
    }

    @Override
    public QuinticSpline getReversed() {

        QuinticSpline spline = new QuinticSpline(
                p5, p4, p3, p2, p1, p0,
                getPathConstraints()
        );
        spline.initialize();
        return spline;
    }

    @Override
    public Pose getPose(double t) {

        if (p0 == null) init();

        double u = 1.0 - t;

        double b0 = u*u*u*u*u;
        double b1 = 5*u*u*u*u*t;
        double b2 = 10*u*u*u*t*t;
        double b3 = 10*u*u*t*t*t;
        double b4 = 5*u*t*t*t*t;
        double b5 = t*t*t*t*t;

        double x =
                b0*p0.getX()
                        + b1*p1.getX()
                        + b2*p2.getX()
                        + b3*p3.getX()
                        + b4*p4.getX()
                        + b5*p5.getX();

        double y =
                b0*p0.getY()
                        + b1*p1.getY()
                        + b2*p2.getY()
                        + b3*p3.getY()
                        + b4*p4.getY()
                        + b5*p5.getY();

        return new Pose(x, y);
    }

    @Override
    public Vector getDerivative(double t) {

        if (p0 == null) init();

        double u = 1.0 - t;

        double dx =
                5*Math.pow(u,4)*(p1.getX()-p0.getX())
                        + 20*Math.pow(u,3)*t*(p2.getX()-p1.getX())
                        + 30*u*u*t*t*(p3.getX()-p2.getX())
                        + 20*u*t*t*t*(p4.getX()-p3.getX())
                        + 5*Math.pow(t,4)*(p5.getX()-p4.getX());

        double dy =
                5*Math.pow(u,4)*(p1.getY()-p0.getY())
                        + 20*Math.pow(u,3)*t*(p2.getY()-p1.getY())
                        + 30*u*u*t*t*(p3.getY()-p2.getY())
                        + 20*u*t*t*t*(p4.getY()-p3.getY())
                        + 5*Math.pow(t,4)*(p5.getY()-p4.getY());

        return new Vector(dx, dy);
    }

    @Override
    public Vector getSecondDerivative(double t) {
        
        if (p0 == null) init();

        double u = 1.0 - t;

        double ddx =
                20*Math.pow(u,3)*(p2.getX()-2*p1.getX()+p0.getX())
                        + 60*u*u*t*(p3.getX()-2*p2.getX()+p1.getX())
                        + 60*u*t*t*(p4.getX()-2*p3.getX()+p2.getX())
                        + 20*Math.pow(t,3)*(p5.getX()-2*p4.getX()+p3.getX());

        double ddy =
                20*Math.pow(u,3)*(p2.getY()-2*p1.getY()+p0.getY())
                        + 60*u*u*t*(p3.getY()-2*p2.getY()+p1.getY())
                        + 60*u*t*t*(p4.getY()-2*p3.getY()+p2.getY())
                        + 20*Math.pow(t,3)*(p5.getY()-2*p4.getY()+p3.getY());

        return new Vector(ddx, ddy);
    }
}
