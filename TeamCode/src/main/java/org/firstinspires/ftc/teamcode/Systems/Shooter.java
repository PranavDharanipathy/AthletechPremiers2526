package org.firstinspires.ftc.teamcode.Systems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Constants.Calculations;
import org.firstinspires.ftc.teamcode.Constants.CameraConstants;
import org.firstinspires.ftc.teamcode.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.Constants.LocalizationConstants;
import org.firstinspires.ftc.teamcode.Constants.Models;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.util.PedroPathing.PoseAcceleration;
import org.firstinspires.ftc.teamcode.util.PedroPathing.PoseVelocity;
import org.firstinspires.ftc.teamcode.util.PedroPathing.PoseSpeedTracker;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.EffectivelySubsystem;

import static org.firstinspires.ftc.teamcode.Constants.CameraConstants.MT1_LOCALIZATION_ELIGIBILITY_MAXIMUM_ROBOT_VELOCITY;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.THC_ENGAGE_VELOCITY;

import androidx.annotation.NonNull;

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

    public enum ZONE {
        CLOSE("CLOSE"), FAR("FAR");

        private String string;

        ZONE(String string) {
            this.string = string;
        }

        @NonNull
        public String toString() {
            return string;
        }
    }

    private ZONE flywheelTargetVelocityZone = ZONE.FAR;

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

    //stage of 0 means that the action is pending, 1 means that it's underway, 2 means that it's complete.
    private int performingAutomaticLocalization = 0;

    private boolean shooterToggle = false;

    private double turretAimPosition;

    private double robotHeadingRad;
    public double tt;

    private Pose goalCoordinate;

    /// For hysteresis control on the turret, this is the robot's position on the field at a point in time in the future.
    public Pose futureRobotPose;
    public Pose currentRobotPose;
    public Pose turretPose;
    private double turretTimeLookahead = 0;
    private boolean shouldUseTHC = false; //initially the bot is stationary

    private double distanceToGoal;

    public void update() {

        poseSpeedTracker.update();

        PoseVelocity robotVelocity = poseSpeedTracker.getPoseVelocity();
        PoseAcceleration robotAcceleration = poseSpeedTracker.getPoseAcceleration();
        double translationalVelocity = Calculations.getRobotTranslationalVelocity(robotVelocity);

        TurretHelper.update(turret);

        if (
                performingAutomaticLocalization == 0
                && translationalVelocity <= MT1_LOCALIZATION_ELIGIBILITY_MAXIMUM_ROBOT_VELOCITY[0]
                && robotVelocity.getAngularVelocity() <= MT1_LOCALIZATION_ELIGIBILITY_MAXIMUM_ROBOT_VELOCITY[1]
        ) {
            camera.update(true, poseEstimator::reset);
            performingAutomaticLocalization = 1;
        }
        else if (performingAutomaticLocalization == 1 && camera.getMt1LocalizationOutcome() == Camera.MT1LocalizationOutcome.FAILED) {
            camera.update(false);
            performingAutomaticLocalization = 0;
        }
        else {
            camera.update(controller1.main_buttonHasJustBeenPressed);
            performingAutomaticLocalization = 2;
        }

        //if (controller2.main_buttonHasJustBeenPressed) relocalization(FieldConstants.RELOCALIZATION_POSE);

        poseEstimator.update(follower, camera);

        currentRobotPose = poseEstimator.getPose();
        robotHeadingRad = currentRobotPose.getHeading();

        double turretCurrentPosition = turret.getCurrentPosition(); //used to calculate turret pose

        //goalAimUpdate();

        //hysteresis control is only used if the robot is moving fast enough
        shouldUseTHC = Math.abs(translationalVelocity) > THC_ENGAGE_VELOCITY[0] || Math.abs(robotVelocity.getAngularVelocity()) > THC_ENGAGE_VELOCITY[1];

        if (false /*shouldUseTHC*/) { //set to 'false /*shouldUseTHC*/' until THC is tuned, after which set to 'shouldUseTHC'

            if (THCTuning) {
                turretTimeLookahead = customTHCTime;
            }
            else {
                turretTimeLookahead = Models.getTHCPosePredictionTime(turretCurrentPosition, turret.getError());
            }

            futureRobotPose = Calculations.getFutureRobotPose(
                    turretTimeLookahead,
                    currentRobotPose,
                    robotVelocity,
                    ShooterConstants.THC_ACCELERATION_INFLUENCE,
                    robotAcceleration
            );
        }
        else {

            turretTimeLookahead = 0;
            futureRobotPose = currentRobotPose;
        }

        turretPose = Calculations.getTurretPoseFromBotPose(futureRobotPose, turretCurrentPosition, turretStartPosition);

        //changing the coordinate that the turret aims at based on targeted zones determined by distance
        if (currentRobotPose.getY() > ShooterConstants.FAR_ZONE_CLOSE_ZONE_BARRIER) {
            goalCoordinate = goalCoordinates.getCloseCoordinate(futureRobotPose.getY(), goalCoordinates);
        }
        else {
            goalCoordinate = goalCoordinates.getFarCoordinate();
        }

        double angleToGoal = Calculations.getAngleToGoal(turretPose.getX(), turretPose.getY(), goalCoordinate);

        double rawtt = angleToGoal - Math.toDegrees(robotHeadingRad);
        tt = Calculations.routeTurret(rawtt);

        turretAimPosition = tt * ShooterConstants.TURRET_TICKS_PER_DEGREE + turretStartPosition;

        turret.setPosition(turretAimPosition);

        //flywheel
        if (controller1.left_bumperHasJustBeenPressed) shooterToggle = !shooterToggle;

        // setting flywheel velocity
        if (controller1.yHasJustBeenPressed) { //close
            flywheelTargetVelocityZone = ZONE.CLOSE;
        }
        else if (controller1.bHasJustBeenPressed) { //far
            flywheelTargetVelocityZone = ZONE.FAR;
        }

        if (shooterToggle) flywheel.setVelocity(getFlywheelTargetVelocity(), true);
        else flywheel.setVelocity(0, true);

        //hood
        hood.setAimZone(
                currentRobotPose.getY() > ShooterConstants.FAR_ZONE_CLOSE_ZONE_BARRIER
                ? Hood.AimZone.CLOSE
                : Hood.AimZone.FAR
        );

        FieldConstants.GoalCoordinatesForDistance goalCoordinatesForDistance =
                goalCoordinates == FieldConstants.GoalCoordinates.BLUE
                        ? FieldConstants.GoalCoordinatesForDistance.BLUE
                        : FieldConstants.GoalCoordinatesForDistance.RED;

        distanceToGoal = Calculations.getDistanceFromGoal(turretPose.getX(), turretPose.getY(), goalCoordinatesForDistance.getCoordinate());

        //updating
        hood.update(distanceToGoal);
        turret.update();
        flywheel.update();

    }

    private double getFlywheelTargetVelocity() {

        double flywheelTargetVelocity;

        if (flywheelTargetVelocityZone == ZONE.FAR) {
            flywheelTargetVelocity = ShooterConstants.FAR_SIDE_FLYWHEEL_SHOOT_VELOCITY;
        }
        else {
            flywheelTargetVelocity = ShooterConstants.CLOSE_SIDE_FLYWHEEL_SHOOT_VELOCITY;
        }
        return flywheelTargetVelocity;
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

    public ZONE getZoneSetting() {
        return flywheelTargetVelocityZone;
    }

    public ZONE getCurrentZoneBasedOnLocation() {
        return currentRobotPose.getY() > ShooterConstants.FAR_ZONE_CLOSE_ZONE_BARRIER ? ZONE.CLOSE : ZONE.FAR;
    }

    public boolean isTurretLookingAhead() {
        return shouldUseTHC;
    }

    public double getTHCLookahead() {
        return turretTimeLookahead;
    }

    private boolean THCTuning = false;

    public void setTHCTuning(boolean tuning) {
        THCTuning = tuning;
    }

    private double customTHCTime;

    public void provideCustomTHCTime(DoubleBinaryOperator model) {
        customTHCTime = model.applyAsDouble(turret.getCurrentPosition(), turret.getError());
    }

}