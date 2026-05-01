package org.firstinspires.ftc.teamcode.util.pedroPathing;

public class PoseAcceleration {

    private double xAcceleration;
    private double yAcceleration;
    private double angAcceleration;
    public PoseAcceleration(double xAcceleration, double yAcceleration, double angAcceleration) {

        this.xAcceleration = xAcceleration;
        this.yAcceleration = yAcceleration;
        this.angAcceleration = angAcceleration;
    }

    public double getXAcceleration() {
        return xAcceleration;
    }

    public double getYAcceleration() {
        return yAcceleration;
    }

    public double getAngularAcceleration() {
        return angAcceleration;
    }

}