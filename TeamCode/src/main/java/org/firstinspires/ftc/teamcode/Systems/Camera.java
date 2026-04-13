package org.firstinspires.ftc.teamcode.Systems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.Calculations;
import org.firstinspires.ftc.teamcode.Constants.CameraConstants;
import org.firstinspires.ftc.teamcode.Constants.Models;
import org.firstinspires.ftc.teamcode.util.pedroPathing.PoseVelocityTracker;
import org.firstinspires.ftc.teamcode.util.BooleanTrigger;
import org.firstinspires.ftc.teamcode.util.LowPassFilter;
import org.firstinspires.ftc.teamcode.util.MathUtil;

public class Camera {


    public enum MT1LocalizationOutcome {
        PENDING, FAILED, SUCCESSFUL, BACKUP;

        public MT1LocalizationOutcome transition(Integer localizationStep) {

            if (localizationStep == null) {
                if (this == PENDING || this == FAILED) {
                    return FAILED;
                }
                else { // this == SUCCESSFUL or this == BACKUP
                    return BACKUP;
                }
            }

            if (localizationStep == 4) {
                return SUCCESSFUL;
            }

            return this;
        }
    }



    private ElapsedTime timer = new ElapsedTime();
    private final Follower follower;
    private final PoseVelocityTracker poseVelocityTracker;
    private final Limelight3A limelight;

    public Limelight3A ll() {
        return limelight;
    }

    public Camera(Follower follower, Limelight3A limelight) {

        this.follower = follower;
        poseVelocityTracker = new PoseVelocityTracker(follower);
        this.limelight = limelight;

        setPollRateHz(CameraConstants.CAMERA_POLL_RATE);
    }

    public static Limelight3A from(HardwareMap hardwareMap, String limelightDeviceName) {
        return hardwareMap.get(Limelight3A.class, limelightDeviceName);
    }

    public void start() {
        limelight.start();
    }

    public void setPollRateHz(int hz) {
        limelight.setPollRateHz(hz);
    }

    private boolean isPipelineSwitched = true;

    private int pipelineIndex;
    public void pipelineSwitch(int index) {

        if (index == pipelineIndex) return;

        pipelineIndex = index;
        isPipelineSwitched = limelight.pipelineSwitch(index);
    }

    public void forcePipelineSwitch(int index) {
        isPipelineSwitched = limelight.pipelineSwitch(index);
    }
    public void reloadPipeline() {
        isPipelineSwitched = limelight.reloadPipeline();
    }

    private double translationalVelMagFromOdo;

    private Pose filteredMT1BotPose;
    private Pose botPoseMT2;
    private boolean hasValidResult;
    private double distanceToTag;

    private boolean isRobotOrientationUpdated;

    private boolean eligibleForMT2 = false;

    public void update(BooleanTrigger localizationTrigger) { // Assumes follower is already updated

        if (!isPipelineSwitched) reloadPipeline();

        poseVelocityTracker.update();

        translationalVelMagFromOdo = Math.abs(Calculations.getRobotTranslationalVelocity(poseVelocityTracker.getPoseVelocity()));

        isRobotOrientationUpdated = false;

        if (eligibleForMT2) {
            isRobotOrientationUpdated = limelight.updateRobotOrientation(Math.toDegrees(follower.getHeading()) + 90);
        }

        LLResult llResult = limelight.getLatestResult();

        hasValidResult = limelight.isConnected() && llResult != null && llResult.isValid();
        if (hasValidResult) {

            distanceToTag = llResult.getBotposeAvgDist();

            if (isRobotOrientationUpdated) {

                botPoseMT2 = Calculations.convertPose3DtoPedroPose(llResult.getBotpose_MT2());
                botPoseMT2 = botPoseMT2.withHeading(MathUtil.normalizeAngleRad(botPoseMT2.getHeading())); //normalizing heading

                if (timer.seconds() >= CameraConstants.ODOMETRY_RELOCALIZATION_FREQUENCY) {
                    follower.setX(botPoseMT2.getX());
                    follower.setY(botPoseMT2.getY());
                    timer.reset();
                }
            }
        }

        relocalizeMT1(localizationTrigger.get());

    }

    private Integer mt1LocalizationStep;

    /// Call after {@link Camera#update}
    /// <p>
    /// Required for MT2
    /// @return MT1 localization step or {@code null} if it failed
    public Integer localizeFollower() {

        LLResult llResult = limelight.getLatestResult();

        if (limelight.isConnected() && llResult != null && llResult.isValid()) {

            Pose mt1Pose = Calculations.convertPose3DtoPedroPose(llResult.getBotpose());

            if (mt1LocalizationStep == null /*failed*/ || mt1LocalizationStep == 4 /*completed*/) {
                filteredMT1BotPose = mt1Pose;
                mt1LocalizationStep = 1;
            }
            else {
                filteredMT1BotPose = filterPose(mt1Pose, translationalVelMagFromOdo);
                mt1LocalizationStep += 1;
            }

            if (mt1LocalizationStep >= CameraConstants.MT1_LOCALIZATION_STEPS) {
                follower.setPose(filteredMT1BotPose);
                eligibleForMT2 = true;
            }
        }
        else mt1LocalizationStep = null;

        return mt1LocalizationStep;
    }

    private boolean localizeMT1 = false;
    private MT1LocalizationOutcome mt1LocalizationOutcome = MT1LocalizationOutcome.PENDING;
    private void relocalizeMT1(boolean trigger) {

        if (trigger) localizeMT1 = true;

        if (localizeMT1) {

            Integer localizationStep = localizeFollower();

            if (localizationStep == null || localizationStep == 4) {
                mt1LocalizationOutcome = mt1LocalizationOutcome.transition(localizationStep);
                localizeMT1 = false;
            }
        }
    }

    private Pose filterPose(Pose rawMT1Pose, double translationalVelMag) {

        double alpha = Models.getScaledMT1BotPoseFilterAlpha(translationalVelMag);

        return new Pose()
                .withX(LowPassFilter.getFilteredValue(filteredMT1BotPose.getX(), rawMT1Pose.getX(), alpha))
                .withY(LowPassFilter.getFilteredValue(filteredMT1BotPose.getY(), rawMT1Pose.getY(), alpha))
                .withHeading(LowPassFilter.getFilteredValue(filteredMT1BotPose.getHeading(), rawMT1Pose.getHeading(), alpha));
    }

    public MT1LocalizationOutcome getMt1LocalizationOutcome() {
        return mt1LocalizationOutcome;
    }

    public Pose getLocalizePose() {
        return filteredMT1BotPose;
    }

    public boolean isEligibleForMT2() {
        return eligibleForMT2;
    }
    public Pose getBotPoseMT2() {
        return botPoseMT2;
    }

    public boolean canUseMT2Pose() {
        return isEligibleForMT2() && hasValidResult() && isRobotOrientationUpdated() && botPoseMT2 != null;
    }

    public boolean hasValidResult() {
        return hasValidResult;
    }

    public double getDistanceToTag() {
        return distanceToTag;
    }

    public boolean isRobotOrientationUpdated() {
        return isRobotOrientationUpdated;
    }

}