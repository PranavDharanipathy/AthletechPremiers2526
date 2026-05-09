package org.firstinspires.ftc.teamcode.Systems;

import static org.firstinspires.ftc.teamcode.Constants.HoodConstants.DISTANCES;
import static org.firstinspires.ftc.teamcode.Constants.HoodConstants.HOOD_POSITIONS;

import org.firstinspires.ftc.teamcode.Constants.HoodConstants;
import org.firstinspires.ftc.teamcode.util.InterpolationData;
import org.firstinspires.ftc.teamcode.util.MathUtil;

public class Hood {

    private HoodAngler hoodAngler;

    public Hood(HoodAngler hoodAngler) {
        this.hoodAngler = hoodAngler;
    }

    private double dInfluence; //basically a kd
    public void setFlywheelVelocityInfluence(double scalar) {
        dInfluence = scalar;
    }

    private Flywheel flywheel;
    private boolean flywheelInitialized = false;

    public void provideFlywheel(Flywheel flywheel) {
        this.flywheel = flywheel;
        flywheelInitialized = true;
    }

    private double d = 0;

    // should be called after flywheel is updated
    public void update(double distanceToGoal) {

        double hoodPosition = getHoodPositionFromInterpolation(distanceToGoal);

        if (flywheelInitialized) {

            double rawD = dInfluence * (flywheel.getTargetVelocity() - flywheel.getCurrentVelocity()) / flywheel.getLoopDt();

            d = Math.abs(rawD) >= HoodConstants.MINIMUM_FLYWHEEL_VELOCITY_HOOD_CORRECTION ? MathUtil.deadband(rawD, d, HoodConstants.FLYWHEEL_VELOCITY_HOOD_CORRECTION_DEADBAND) : 0;

            hoodPosition+=d;
        }

        hoodAngler.setSafePosition(hoodPosition);

    }

    private double getHoodPositionFromInterpolation(double distanceToGoal) {

        if (distanceToGoal < DISTANCES.get(0)) {
            return HOOD_POSITIONS.get(0);
        }
        else if (distanceToGoal > DISTANCES.get(DISTANCES.size() - 1)) {
            return HOOD_POSITIONS.get(HOOD_POSITIONS.size() - 1);
        }

        //converting list to array - same positions are used for p and d interpolations
        double[] distances = DISTANCES.stream().mapToDouble(Double::doubleValue).toArray();

        //getting bounds of the current target position
        double[] bounds = MathUtil.findBoundingValues(distances, distanceToGoal);

        double distance0 = bounds[0];
        double distance1 = bounds[1];

        double hoodPosition0 = HOOD_POSITIONS.get(DISTANCES.indexOf(distance0));
        double hoodPosition1 = HOOD_POSITIONS.get(DISTANCES.indexOf(distance1));

        //returning kd
        return MathUtil.interpolateLinear(

                distanceToGoal,

                new InterpolationData(
                        new double[] {distance0, hoodPosition0},
                        new double[] {distance1, hoodPosition1}
                )
        );

    }
}
