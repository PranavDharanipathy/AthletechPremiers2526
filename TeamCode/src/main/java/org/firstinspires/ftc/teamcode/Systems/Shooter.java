package org.firstinspires.ftc.teamcode.Systems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Constants.Calculations;
import org.firstinspires.ftc.teamcode.Constants.CameraConstants;
import org.firstinspires.ftc.teamcode.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;
import org.firstinspires.ftc.teamcode.Constants.LocalizationConstants;
import org.firstinspires.ftc.teamcode.Constants.Models;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.util.InterpolationData;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.PedroPathing.PoseAcceleration;
import org.firstinspires.ftc.teamcode.util.PedroPathing.PoseVelocity;
import org.firstinspires.ftc.teamcode.util.PedroPathing.PoseSpeedTracker;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.EffectivelySubsystem;

import static org.firstinspires.ftc.teamcode.Constants.CameraConstants.MT1_LOCALIZATION_ELIGIBILITY_MAXIMUM_ROBOT_VELOCITY;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.CLOSE_FLYWHEEL_VELOCITIES;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.CLOSE_HOOD_DISTANCES;

import java.util.List;
import java.util.function.DoubleBinaryOperator;

public class Shooter implements EffectivelySubsystem {

    private BetterGamepad controller1;

    public Flywheel flywheel;

    public TurretBase turret;

    public Hood hood;

    private Follower follower;
    public Camera camera;
    public PoseSpeedTracker poseSpeedTracker;

    public PoseEstimator poseEstimator;

    public void provideComponents(Flywheel flywheel, TurretBase turret, HoodAngler hoodAngler, Follower follower, Camera unstartedCamera, BetterGamepad controller1) {

        this.follower = follower;
        camera = unstartedCamera;
        poseSpeedTracker = new PoseSpeedTracker(follower);

        this.flywheel = flywheel;

        this.turret = turret;

        hood = new Hood(hoodAngler);
        hood.setFlywheelVelocityAdjustmentParameters(
                ShooterConstants.FLYWHEEL_VELOCITY_HOOD_CORRECTION_INFLUENCE,
                ShooterConstants.FLYWHEEL_VELOCITY_HOOD_CORRECTION_MINIMUM,
                ShooterConstants.FLYWHEEL_VELOCITY_HOOD_CORRECTION_MAXIMUM
        );
        hood.provideFlywheel(this.flywheel);

        poseEstimator = new PoseEstimator(
                follower.getPose(),
                LocalizationConstants.ODOMETRY_STD_DEV,
                LocalizationConstants.CAMERA_STD_DEV,
                LocalizationConstants.PROCESS_NOISE_STD_DEV
        );

        this.controller1 = controller1;

    }

    private double turretStartPosition;

    private FieldConstants.GoalCoordinates goalCoordinates;

    /// Primarily for modification purposes, however can totally be used for telemetry, haptics, etc.
    public FieldConstants.GoalCoordinates accessGoalCoordinates() {
        return goalCoordinates;
    }

    private CurrentAlliance.ALLIANCE alliance;

    public void switchAlliance(CurrentAlliance.ALLIANCE alliance) {

        this.alliance = alliance;

        if (this.alliance == CurrentAlliance.ALLIANCE.BLUE_ALLIANCE) {
            goalCoordinates = FieldConstants.GoalCoordinates.BLUE;
        }
        else {
            goalCoordinates = FieldConstants.GoalCoordinates.RED;
        }

    }

    public void start(CurrentAlliance.ALLIANCE alliance) {

        camera.start();

        this.alliance = alliance;

        goalCoordinates = this.alliance == CurrentAlliance.ALLIANCE.BLUE_ALLIANCE ? FieldConstants.GoalCoordinates.BLUE : FieldConstants.GoalCoordinates.RED;

        camera.pipelineSwitch(CameraConstants.PIPELINES.GENERAL_GOAL_PIPELINE.getPipelineIndex());

        turretAimPosition = turretStartPosition = turret.startPosition;

        camera.start();

        flywheel.reset();
    }

    private boolean shooterToggle = false;

    private double turretAimPosition;

    private double robotHeadingRad;
    public double tt;

    private Pose goalCoordinate;

    public Pose currentRobotPose;
    public Pose turretPose;

    public double distanceToGoal;

    private FieldConstants.GoalCoordinatesForDistance goalCoordinatesForDistance;

    public void update() {

        poseSpeedTracker.update();

        PoseVelocity robotVelocity = poseSpeedTracker.getPoseVelocity();
        PoseAcceleration robotAcceleration = poseSpeedTracker.getPoseAcceleration();
        double translationalVelocity = Calculations.getRobotTranslationalVelocity(robotVelocity);

        if (
                !camera.isEligibleForMT2()
                && translationalVelocity <= MT1_LOCALIZATION_ELIGIBILITY_MAXIMUM_ROBOT_VELOCITY[0]
                && robotVelocity.getAngularVelocity() <= MT1_LOCALIZATION_ELIGIBILITY_MAXIMUM_ROBOT_VELOCITY[1]
        ) {
            camera.update(true, poseEstimator::reset);
        }
        else {
            camera.update(controller1.main_buttonHasJustBeenPressed);
        }

        if (camera.hasJustRunMT1Localization()) controller1.rumble(GeneralConstants.NORMAL_CONTROLLER_RUMBLE_TIME);

        //if (controller2.main_buttonHasJustBeenPressed) relocalization(FieldConstants.RELOCALIZATION_POSE);

        poseEstimator.update(follower, camera);

        currentRobotPose = poseEstimator.getPose();
        robotHeadingRad = currentRobotPose.getHeading();

        double turretCurrentPosition = turret.getCurrentPosition(); //used to calculate turret pose

        //goalAimUpdate();

        turretPose = Calculations.getTurretPoseFromBotPose(currentRobotPose, turretCurrentPosition, turretStartPosition);

        //hood
        hood.setAimZone(
                currentRobotPose.getY() > ShooterConstants.FAR_ZONE_CLOSE_ZONE_BARRIER
                ? Hood.AimZone.CLOSE
                : Hood.AimZone.FAR
        );

        goalCoordinatesForDistance =
                goalCoordinates == FieldConstants.GoalCoordinates.BLUE
                        ? FieldConstants.GoalCoordinatesForDistance.BLUE
                        : FieldConstants.GoalCoordinatesForDistance.RED;

        distanceToGoal = Calculations.getDistanceFromGoal(turretPose.getX(), turretPose.getY(), goalCoordinatesForDistance.getCoordinate());

        //flywheel
        if (controller1.left_bumperHasJustBeenPressed) shooterToggle = !shooterToggle;

        if (shooterToggle) flywheel.setVelocity(getFlywheelTargetVelocityFromInterpolation(currentRobotPose, robotVelocity, robotAcceleration), false);
        else flywheel.setVelocity(0, true);

        //turret

        double flywheelTargetVelocity = flywheel.getTargetVelocity();
        double timeOfFlight = flywheelTargetVelocity != 0 ? distanceToGoal / Models.getBallSpeedFromFlywheel(flywheelTargetVelocity) : 0;

        //changing the coordinate that the turret aims at based on targeted zones determined by distance
        if (currentRobotPose.getY() > ShooterConstants.FAR_ZONE_CLOSE_ZONE_BARRIER) {
            goalCoordinate = goalCoordinates.getCloseCoordinate(currentRobotPose.getY(), goalCoordinates);
        }
        else {
            goalCoordinate = goalCoordinates.getFarCoordinate();
        }

        Pose virtualGoal = Calculations.getVirtualGoalCoordinate(timeOfFlight, robotVelocity, robotAcceleration, goalCoordinate);

        double angleToGoal = Calculations.getAngleToGoal(turretPose.getX(), turretPose.getY(), virtualGoal);

        double rawtt = MathUtil.normalizeAngleDeg(Math.toDegrees(robotHeadingRad) - angleToGoal);
        tt = Calculations.routeTurret(rawtt);

        turretAimPosition = tt * ShooterConstants.TURRET_TICKS_PER_DEGREE + turretStartPosition;

        turret.setAim(turretAimPosition, virtualGoal, currentRobotPose, robotVelocity);
        //turret.setAim(turretAimPosition, goalCoordinate, currentRobotPose, robotVelocity);


        //updating
        flywheel.update();
        hood.update(distanceToGoal);
        turret.update();
    }

    private double getFlywheelTargetVelocityFromInterpolation(Pose botPose, PoseVelocity robotVelocity, PoseAcceleration robotAcceleration) {

        Pose futurePose = Calculations.getFutureBotPose(ShooterConstants.FLYWHEEL_SPEED_ADJUSTMENT_T, botPose, robotVelocity, robotAcceleration);

        double distanceToGoal = Calculations.getDistanceFromGoal(futurePose.getX(), futurePose.getY(), goalCoordinatesForDistance.getCoordinate());

        List<Double> distances = CLOSE_HOOD_DISTANCES;
        List<Double> velocities = CLOSE_FLYWHEEL_VELOCITIES;

        if (distanceToGoal < distances.get(0)) {
            return velocities.get(0);
        }
        else if (distanceToGoal > distances.get(distances.size() - 1)) {
            return velocities.get(velocities.size() - 1);
        }

        double[] distancesArray = distances.stream().mapToDouble(Double::doubleValue).toArray();

        double[] bounds = MathUtil.findBoundingValues(distancesArray, distanceToGoal);

        double distance0 = bounds[0];
        double distance1 = bounds[1];

        double flywheelVelocity0 = velocities.get(distances.indexOf(distance0));
        double flywheelVelocity1 = velocities.get(distances.indexOf(distance1));

        return MathUtil.interpolateLinear(

                distanceToGoal,

                new InterpolationData(
                        new double[] {distance0, flywheelVelocity0},
                        new double[] {distance1, flywheelVelocity1}
                )
        );

    }

    private void goalAimUpdate() {

//        if (controller2.dpad_leftHasJustBeenPressed) {
//            turretStartPosition+=ShooterConstants.TURRET_HOME_POSITION_INCREMENT;
//        }
//        else if (controller2.dpad_rightHasJustBeenPressed) {
//            turretStartPosition-=ShooterConstants.TURRET_HOME_POSITION_INCREMENT;
//        }

//        if (alliance == CurrentAlliance.ALLIANCE.BLUE_ALLIANCE) {
//            goalPositionalIncrementBlue(controller2);
//        }
//        else {
//            goalPositionalIncrementRed(controller2);
//        }
    }

    private void goalPositionalIncrementRed(BetterGamepad controller) {
        if (controller.dpad_leftHasJustBeenPressed) {
            goalCoordinates.incrementAll(ShooterConstants.GOAL_X_POSITION_INCREMENT, 0);
        }
        else if (controller.dpad_rightHasJustBeenPressed) {
            goalCoordinates.incrementAll(-ShooterConstants.GOAL_X_POSITION_INCREMENT, 0);
        }

        if (controller.dpad_upHasJustBeenPressed) {
            goalCoordinates.incrementAll(0, -ShooterConstants.GOAL_Y_POSITION_INCREMENT);
        }
        else if (controller.dpad_downHasJustBeenPressed) {
            goalCoordinates.incrementAll(0, ShooterConstants.GOAL_Y_POSITION_INCREMENT);
        }
    }

    private void goalPositionalIncrementBlue(BetterGamepad controller) {
        if (controller.dpad_leftHasJustBeenPressed) {
            goalCoordinates.incrementAll(-ShooterConstants.GOAL_X_POSITION_INCREMENT, 0);
        }
        else if (controller.dpad_rightHasJustBeenPressed) {
            goalCoordinates.incrementAll(ShooterConstants.GOAL_X_POSITION_INCREMENT, 0);
        }

        if (controller.dpad_upHasJustBeenPressed) {
            goalCoordinates.incrementAll(0, ShooterConstants.GOAL_Y_POSITION_INCREMENT);
        }
        else if (controller.dpad_downHasJustBeenPressed) {
            goalCoordinates.incrementAll(0, -ShooterConstants.GOAL_Y_POSITION_INCREMENT);
        }
    }

    private void relocalization(Pose reZeroPose) {

        follower.setPose(reZeroPose);
        poseEstimator.reset(reZeroPose);
    }

    public String getCurrentZoneBasedOnLocation() {
        return currentRobotPose.getY() > ShooterConstants.FAR_ZONE_CLOSE_ZONE_BARRIER ? "CLOSE" : "FAR";
    }

}