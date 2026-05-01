package org.firstinspires.ftc.teamcode.util.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import java.util.ArrayList;
import java.util.List;

public class PoseSpeedTracker {

    private Follower follower;

    public PoseSpeedTracker(Follower follower) {

        this.follower = follower;

        xSpeedHistory.ensureCapacity(4);
        ySpeedHistory.ensureCapacity(4);
        angSpeedHistory.ensureCapacity(4);
    }

    // (velocity) index 0 is previous and index 1 is current
    // (acceleration) index 2 is previous and index 3 is current
    private ArrayList<Double> xSpeedHistory = new ArrayList<>(List.of(0.0, 0.0, 0.0, 0.0));
    private ArrayList<Double> ySpeedHistory = new ArrayList<>(List.of(0.0, 0.0, 0.0, 0.0));
    private ArrayList<Double> angSpeedHistory = new ArrayList<>(List.of(0.0, 0.0, 0.0, 0.0));

    private void buildVelHistory(List<Double> velHistory, double currentVel) {

        velHistory.set(0, velHistory.get(1));
        velHistory.set(1, currentVel);
    }

    private double calcVelUnitless(List<Double> velHistory) {

        double[] history = velHistory.stream().mapToDouble(Double::doubleValue).toArray();

        return history[1] - history[0];
    }

    private void buildAccelHistory(List<Double> accelHistory, double currentAccel) {

        accelHistory.set(2, accelHistory.get(3));
        accelHistory.set(3, currentAccel);
    }

    private double calcAccelUnitless(List<Double> accelHistory) {

        double[] history = accelHistory.stream().mapToDouble(Double::doubleValue).toArray();

        return history[3] - history[2];
    }

    private double xVelocity, yVelocity, angularVelocity;

    private double xAcceleration, yAcceleration, angularAcceleration;

    private double getSeconds() {
        return System.nanoTime() * 1e-9;
    }
    private double prevTime, currTime;

    public void update() {

        prevTime = currTime;
        currTime = getSeconds();

        double dt = currTime - prevTime;

        Pose pose = follower.getPose(); //follower must have been updated

        buildVelHistory(xSpeedHistory, pose.getX());
        buildVelHistory(ySpeedHistory, pose.getY());
        buildVelHistory(angSpeedHistory, pose.getHeading());

        xVelocity = calcVelUnitless(xSpeedHistory) / dt;
        yVelocity = calcVelUnitless(ySpeedHistory) / dt;
        angularVelocity = calcVelUnitless(angSpeedHistory) / dt;

        buildAccelHistory(xSpeedHistory, xVelocity);
        buildAccelHistory(ySpeedHistory, yVelocity);
        buildAccelHistory(angSpeedHistory, angularVelocity);

        xAcceleration = calcAccelUnitless(xSpeedHistory) / dt;
        yAcceleration = calcAccelUnitless(ySpeedHistory) / dt;
        angularAcceleration = calcAccelUnitless(angSpeedHistory) / dt;
    }

    /// <p>x: in/sec </p>
    /// <p>y: in/sec </p>
    /// <p>heading: rad/sec </p>
    public PoseVelocity getPoseVelocity() {
        return new PoseVelocity(xVelocity, yVelocity, angularVelocity);
    }

    /// <p>x: in/sec </p>
    /// <p>y: in/sec </p>
    /// <p>heading: rad/sec </p>
    public PoseAcceleration getPoseAcceleration() {
        return new PoseAcceleration(xAcceleration, yAcceleration, angularAcceleration);
    }

    /// Index 0 is xVelHistory, index 1 is yVelHistory, index 2 is angVelHistory
    public double[][] getHistories() {

        double[] xH = xSpeedHistory.stream().mapToDouble(Double::doubleValue).toArray();
        double[] yH = ySpeedHistory.stream().mapToDouble(Double::doubleValue).toArray();
        double[] angH = angSpeedHistory.stream().mapToDouble(Double::doubleValue).toArray();

        return new double[][] {xH, yH, angH};
    }
}