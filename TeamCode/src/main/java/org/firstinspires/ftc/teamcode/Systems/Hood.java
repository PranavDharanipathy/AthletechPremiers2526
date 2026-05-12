package org.firstinspires.ftc.teamcode.Systems;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.util.InterpolationData;
import org.firstinspires.ftc.teamcode.util.MathUtil;

import java.util.List;

public class Hood {

    private HoodAngler hoodAngler;

    public HoodAngler accessHoodAngler() {
        return hoodAngler;
    }

    public Hood(HoodAngler hoodAngler) {
        this (hoodAngler, AimZone.CLOSE);
    }

    public Hood(HoodAngler hoodAngler, AimZone startingAimZone) {
        this.hoodAngler = hoodAngler;
        setAimZone(startingAimZone);
    }

    private double dInfluence; //basically a kd
    private double dMin, dMax;

    /// @param dInfluence Flywheel velocity influence
    /// @param dMin Minimum flywheel velocity based hood correction
    /// @param dMax Maximum flywheel velocity based hood correction
    public void setFlywheelVelocityAdjustmentParameters(double dInfluence, double dMin, double dMax) {

        this.dInfluence = dInfluence;
        this.dMin = dMin;
        this.dMax = dMax;
    }

    private Flywheel flywheel;
    private boolean flywheelInitialized = false;

    public void provideFlywheel(Flywheel flywheel) {
        this.flywheel = flywheel;
        flywheelInitialized = true;
    }

    public enum AimZone {
        CLOSE, FAR
    }

    private AimZone aimZone;

    public void setAimZone(AimZone aimZone) {
        this.aimZone = aimZone;
    }

    private double d = 0;

    /// should be called after flywheel is updated
    public void update(double distanceToGoal) {

        double hoodPosition = getHoodPositionFromInterpolation(
                distanceToGoal,
                aimZone == AimZone.CLOSE ? ShooterConstants.CLOSE_HOOD_DISTANCES : ShooterConstants.FAR_HOOD_DISTANCES,
                aimZone == AimZone.CLOSE ? ShooterConstants.CLOSE_HOOD_POSITIONS : ShooterConstants.FAR_HOOD_POSITIONS
        );

        if (flywheelInitialized) {

            double rawD = dInfluence * (flywheel.getTargetVelocity() - flywheel.getCurrentVelocity()) / flywheel.getLoopDt();

            d = Math.abs(rawD) >= ShooterConstants.MINIMUM_FLYWHEEL_VELOCITY_HOOD_CORRECTION ? MathUtil.deadband(rawD, d, ShooterConstants.FLYWHEEL_VELOCITY_HOOD_CORRECTION_DEADBAND) : 0;
            d = MathUtil.clamp(d, dMin, dMax);

            hoodPosition+=getCorrectiveD(d);
        }

        hoodAngler.setSafePosition(hoodPosition);

    }

    /// should be called after flywheel is updated
    public void tuningUpdate(double distanceToGoal, List<Double> distances, List<Double> positions) {

        if (distances.size() < 2 || positions.size() < 2) return;

        double hoodPosition = getHoodPositionFromInterpolation(distanceToGoal, distances, positions);

        if (flywheelInitialized) {

            double rawD = dInfluence * (flywheel.getTargetVelocity() - flywheel.getCurrentVelocity()) / flywheel.getLoopDt();

            d = Math.abs(rawD) >= ShooterConstants.MINIMUM_FLYWHEEL_VELOCITY_HOOD_CORRECTION ? MathUtil.deadband(rawD, d, ShooterConstants.FLYWHEEL_VELOCITY_HOOD_CORRECTION_DEADBAND) : 0;

            hoodPosition+=getCorrectiveD(d);
        }

        hoodAngler.setSafePosition(hoodPosition);

    }

    private double getCorrectiveD(double d) {

        double increasedShotDistanceDirection = hoodAngler.getPosition() <= ShooterConstants.HOOD_POSITION_FOR_MAX_SHOT_DISTANCE ? 1 : -1;

        if (d >= 0) { //increase shot distance
            return d * increasedShotDistanceDirection;
        }
        else { //decrease shot distance
            return d * -increasedShotDistanceDirection;
        }
    }

    private double getHoodPositionFromInterpolation(double distanceToGoal, List<Double> distancesList, List<Double> positionsList) {

        if (distanceToGoal < distancesList.get(0)) {
            return positionsList.get(0);
        }
        else if (distanceToGoal > distancesList.get(distancesList.size() - 1)) {
            return positionsList.get(positionsList.size() - 1);
        }

        //converting list to array - same positions are used for p and d interpolations
        double[] distances = distancesList.stream().mapToDouble(Double::doubleValue).toArray();

        //getting bounds of the current target position
        double[] bounds = MathUtil.findBoundingValues(distances, distanceToGoal);

        double distance0 = bounds[0];
        double distance1 = bounds[1];

        double hoodPosition0 = positionsList.get(distancesList.indexOf(distance0));
        double hoodPosition1 = positionsList.get(distancesList.indexOf(distance1));

        //returning kd
        return MathUtil.interpolateLinear(

                distanceToGoal,

                new InterpolationData(
                        new double[] {distance0, hoodPosition0},
                        new double[] {distance1, hoodPosition1}
                )
        );

    }

    public int getCorrectiveDirection() {
        return (int) getCorrectiveD(1);
    }
}
