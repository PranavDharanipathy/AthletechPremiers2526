package org.firstinspires.ftc.teamcode.Systems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Constants.Calculations;
import org.firstinspires.ftc.teamcode.Constants.CameraConstants;
import org.firstinspires.ftc.teamcode.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;
import org.firstinspires.ftc.teamcode.Constants.Models;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.util.pedroPathing.PoseVelocity;
import org.firstinspires.ftc.teamcode.util.pedroPathing.PoseVelocityTracker;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.BooleanTrigger;
import org.firstinspires.ftc.teamcode.util.EffectivelySubsystem;

import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.TURRET_HYSTERESIS_CONTROL_ENGAGE_VELOCITY;

import androidx.annotation.NonNull;

import java.util.function.DoubleBinaryOperator;

public class Shooter implements EffectivelySubsystem {

    private BetterGamepad controller1, controller2;

    public Flywheel flywheel;

    public TurretBase turret;

    public HoodAngler hoodAngler;

    private Follower follower;
    public Camera camera;
    private PoseVelocityTracker poseVelocityTracker;

    public void provideComponents(Flywheel flywheel, TurretBase turret, HoodAngler hoodAngler, Follower follower, Camera unstartedCamera, BetterGamepad controller1, BetterGamepad controller2) {

        this.follower = follower;
        camera = unstartedCamera;
        poseVelocityTracker = new PoseVelocityTracker(follower);

        this.flywheel = flywheel;

        this.turret = turret;

        this.hoodAngler = hoodAngler;

        this.controller1 = controller1;
        this.controller2 = controller2;

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

    private final BooleanTrigger needToAttemptRelocalizationAtStartOfTeleOp = new BooleanTrigger(true);

    private boolean shooterToggle = false;

    private double hoodPosition;

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

        //getting robot pose/vel data
        if (controller2.shareHasJustBeenPressed) camera.reloadPipeline();
        camera.update(new BooleanTrigger(controller2.main_buttonHasJustBeenPressed).or(needToAttemptRelocalizationAtStartOfTeleOp));
        if (needToAttemptRelocalizationAtStartOfTeleOp.get()) needToAttemptRelocalizationAtStartOfTeleOp.set(false);

        if (controller2.yHasJustBeenPressed) relocalization(FieldConstants.RELOCALIZATION_POSE);

        poseVelocityTracker.update();
        TurretHelper.update(turret);

        currentRobotPose = camera.canUseMT2Pose() ? camera.getBotPoseMT2() : follower.getPose();
        robotHeadingRad = currentRobotPose.getHeading();
        PoseVelocity robotVelocity = poseVelocityTracker.getPoseVelocity();
        double translationalVelocity = Calculations.getRobotTranslationalVelocity(robotVelocity.getXVelocity(), robotVelocity.getYVelocity());
        //turret
        double turretCurrentPosition = turret.getCurrentPosition(); //used to calculate turret pose

        goalAimUpdate();

        //hysteresis control is only used if the robot is moving fast enough
        shouldUseTHC = Math.abs(translationalVelocity) > TURRET_HYSTERESIS_CONTROL_ENGAGE_VELOCITY[0] || Math.abs(robotVelocity.getAngularVelocity()) > TURRET_HYSTERESIS_CONTROL_ENGAGE_VELOCITY[1];

        if (false /*shouldUseTHC*/) {

            if (THCTuning) {
                turretTimeLookahead = customTHCTime;
            }
            else {
                turretTimeLookahead = Models.getTHCPosePredictionTime(turretCurrentPosition, turret.getError());
            }

            futureRobotPose = Calculations.getFutureRobotPose(
                    turretTimeLookahead,
                    currentRobotPose,
                    robotVelocity
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

        double angleToGoal;

        angleToGoal = Calculations.getAngleToGoal(turretPose.getX(), turretPose.getY(), goalCoordinate);

        double rawtt = angleToGoal - Math.toDegrees(robotHeadingRad);
        tt = Calculations.routeTurret(rawtt);

        turretAimPosition = tt * ShooterConstants.TURRET_TICKS_PER_DEGREE + turretStartPosition;

        double turretTargetPosition;
        if (controller2.left_trigger(GeneralConstants.TRIGGER_THRESHOLD)) turretTargetPosition = turretStartPosition;
        else turretTargetPosition = turretAimPosition;

        turret.setPosition(turretTargetPosition);

        //flywheel
        if (controller1.left_bumperHasJustBeenPressed) shooterToggle = !shooterToggle;

        // setting flywheel velocity
        if (controller1.yHasJustBeenPressed) { //close

            flywheelTargetVelocityZone = ZONE.CLOSE;
            controller2.rumble(GeneralConstants.NORMAL_CONTROLLER_RUMBLE_TIME);
        }
        else if (controller1.bHasJustBeenPressed) { //far

            flywheelTargetVelocityZone = ZONE.FAR;
            controller2.rumble(GeneralConstants.NORMAL_CONTROLLER_RUMBLE_TIME);
        }

        if (shooterToggle) flywheel.setVelocity(getFlywheelTargetVelocity(), true);
        else flywheel.setVelocity(0, true);

        //hood
        if (flywheelTargetVelocityZone == ZONE.FAR) hoodPosition = ShooterConstants.HOOD_FAR_POSITION;
        else {

            FieldConstants.GoalCoordinatesForDistance goalCoordinatesForDistance =
                    goalCoordinates == FieldConstants.GoalCoordinates.BLUE
                            ? FieldConstants.GoalCoordinatesForDistance.BLUE
                            : FieldConstants.GoalCoordinatesForDistance.RED;

            distanceToGoal = Calculations.getDistanceFromGoal(turretPose.getX(), turretPose.getY(), goalCoordinatesForDistance.getCoordinate());

            hoodPosition = Models.getCloseHoodPosition(distanceToGoal);
        }

        //updating
        hoodAngler.setSafePosition(hoodPosition);
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

        if (alliance == CurrentAlliance.ALLIANCE.BLUE_ALLIANCE) {
            goalPositionalIncrementBlue();
        }
        else {
            goalPositionalIncrementRed();
        }
    }

    private void goalPositionalIncrementRed() {
        if (controller2.dpad_leftHasJustBeenPressed) {
            goalCoordinates.incrementAll(ShooterConstants.GOAL_X_POSITION_INCREMENT, 0);
        }
        else if (controller2.dpad_rightHasJustBeenPressed) {
            goalCoordinates.incrementAll(-ShooterConstants.GOAL_X_POSITION_INCREMENT, 0);
        }

        if (controller2.dpad_upHasJustBeenPressed) {
            goalCoordinates.incrementAll(0, -ShooterConstants.GOAL_Y_POSITION_INCREMENT);
        }
        else if (controller2.dpad_downHasJustBeenPressed) {
            goalCoordinates.incrementAll(0, ShooterConstants.GOAL_Y_POSITION_INCREMENT);
        }
    }

    private void goalPositionalIncrementBlue() {
        if (controller2.dpad_leftHasJustBeenPressed) {
            goalCoordinates.incrementAll(-ShooterConstants.GOAL_X_POSITION_INCREMENT, 0);
        }
        else if (controller2.dpad_rightHasJustBeenPressed) {
            goalCoordinates.incrementAll(ShooterConstants.GOAL_X_POSITION_INCREMENT, 0);
        }

        if (controller2.dpad_upHasJustBeenPressed) {
            goalCoordinates.incrementAll(0, ShooterConstants.GOAL_Y_POSITION_INCREMENT);
        }
        else if (controller2.dpad_downHasJustBeenPressed) {
            goalCoordinates.incrementAll(0, -ShooterConstants.GOAL_Y_POSITION_INCREMENT);
        }
    }

    private void relocalization(Pose reZeroPose) {
        follower.setPose(reZeroPose);
    }

    public ZONE getZone() {
        return flywheelTargetVelocityZone;
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