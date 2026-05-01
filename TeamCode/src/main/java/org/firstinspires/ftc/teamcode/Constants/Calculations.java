package org.firstinspires.ftc.teamcode.Constants;

import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.THC_ACCELERATION_THRESHOLD;

import com.pedropathing.geometry.Pose;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.util.pedroPathing.PoseAcceleration;
import org.firstinspires.ftc.teamcode.util.pedroPathing.PoseVelocity;
import org.firstinspires.ftc.teamcode.util.MathUtil;

public class Calculations {

    public static Pose convertPose3DtoPedroPose(Pose3D pose3d) {

        double x = MathUtil.metersToInches(pose3d.getPosition().y);
        double y = -MathUtil.metersToInches(pose3d.getPosition().x);
        double yaw = Math.toRadians(pose3d.getOrientation().getYaw() - 90);

        return new Pose(x, y, yaw);
    }

    /// The angle in degrees that is required for any system to look in to be pointing at the goal.
    /// <p>
    /// x is forward-backward and y is left-right.
    /// @param x In inches
    /// @param y In inches
    public static double getAngleToGoal(double x, double y, Pose goalCoordinate) {

        double dx = goalCoordinate.getX() - x;
        double dy = goalCoordinate.getY() - y;

        return Math.toDegrees(FastMath.atan2(dy, dx));
    }

    /// Gets the flat (2d) distance from the goal.
    public static double getDistanceFromGoal(double x, double y, Pose goalCoordinate) {
        return MathUtil.getDistance2d(x, goalCoordinate.getX(), y, goalCoordinate.getY());
    }

    /// @return Turret's x, y, and heading absolute to the field
    public static Pose getTurretPoseFromBotPose(Pose botPose, double turretPositionTicks, double turretStartPositionTicks) {

        double reZeroedTurretTicks = turretPositionTicks - turretStartPositionTicks;
        double turretRotation = Math.toRadians(reZeroedTurretTicks / ShooterConstants.TURRET_TICKS_PER_DEGREE);

        double turretHeading = botPose.getHeading() + turretRotation;

        double turretX = botPose.getX() + (ShooterConstants.TURRET_POSITIONAL_OFFSET * FastMath.sin(botPose.getHeading()));
        double turretY = botPose.getY() + (ShooterConstants.TURRET_POSITIONAL_OFFSET * FastMath.cos(botPose.getHeading()));

        return new Pose(
                turretX,
                turretY,
                turretHeading
        );
    }

    /// @param accelerationInfluence Scalar value representing how influential acceleration is.
    public static Pose getFutureRobotPose(double t, Pose currentRobotPose, PoseVelocity poseVelocity, double accelerationInfluence, PoseAcceleration poseAcceleration) {

        final double translationalAccel = Math.hypot(poseAcceleration.getXAcceleration(), poseAcceleration.getYAcceleration());

        double translationalAccelInfluence = translationalAccel >= THC_ACCELERATION_THRESHOLD[0] ? accelerationInfluence : 0;
        double headingAccelInfluence = poseAcceleration.getAngularAcceleration() >= THC_ACCELERATION_THRESHOLD[1] ? accelerationInfluence : 0;

        return new Pose(
                currentRobotPose.getX() + (t * poseVelocity.getXVelocity()) + (translationalAccelInfluence * poseAcceleration.getXAcceleration() * (t * t)),
                currentRobotPose.getY() + (t * poseVelocity.getYVelocity()) + (translationalAccelInfluence * poseAcceleration.getYAcceleration() * (t * t)),
                MathUtil.normalizeAngleRad(currentRobotPose.getHeading() + (t * poseVelocity.getAngularVelocity()) + (headingAccelInfluence * poseAcceleration.getAngularAcceleration() * (t * t)))
        );
    }


    /// @return robots translational velocity vector
    public static double getRobotTranslationalVelocity(double xVelocity, double yVelocity) {
        return Math.hypot(xVelocity, yVelocity);
    }

    /// @return robots translational velocity vector
    public static double getRobotTranslationalVelocity(PoseVelocity poseVelocity) {
        return Math.hypot(poseVelocity.getXVelocity(), poseVelocity.getYVelocity());
    }

    public static double routeTurret(double rawtt) {

        if (rawtt >= ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES && rawtt <= ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES) return rawtt; //no need to reroute

        double[] reroutes = {rawtt - 360, rawtt + 360};
        if (reroutes[0] >= ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES && reroutes[0] <= ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES) {
            return reroutes[0];
        }
        else if (reroutes[1] >= ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES && reroutes[1] <= ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES) {
            return reroutes[1];
        }

        // go to the closest limit if target position is outside the min and max
        else if (rawtt < ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES) return ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES;
        else return ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES;
    }

}