package org.firstinspires.ftc.teamcode.Auto.autosubsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants.Calculations;
import org.firstinspires.ftc.teamcode.Constants.ConfigurationConstants;
import org.firstinspires.ftc.teamcode.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.Constants.MapSetterConstants;
import org.firstinspires.ftc.teamcode.Constants.Models;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Systems.CurrentAlliance;
import org.firstinspires.ftc.teamcode.Systems.Flywheel;
import org.firstinspires.ftc.teamcode.Systems.Hood;
import org.firstinspires.ftc.teamcode.Systems.HoodAngler;
import org.firstinspires.ftc.teamcode.Systems.Shooter;
import org.firstinspires.ftc.teamcode.Systems.TurretBase;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.PedroPathing.PoseSpeedTracker;
import org.firstinspires.ftc.teamcode.util.PedroPathing.PoseVelocity;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;


public class ShooterNF implements Subsystem {

    public ShooterNF() {}

    public static ShooterNF INSTANCE = new ShooterNF();

    public Flywheel flywheel;
    public Hood hood;
    public TurretBase turret;

    private PoseSpeedTracker poseSpeedTracker;
    private Follower follower;

    private boolean followerInitialized = false;
    public void provideFollower(Follower follower) {

        this.follower = follower;
        poseSpeedTracker = new PoseSpeedTracker(this.follower);

        followerInitialized = true;
    }

    private FieldConstants.GoalCoordinates goalCoordinates;

    private boolean goalCoordinatesInitialized = false;
    public void provideAlliance(CurrentAlliance.ALLIANCE alliance) {

        goalCoordinates = alliance == CurrentAlliance.ALLIANCE.BLUE_ALLIANCE ? FieldConstants.GoalCoordinates.BLUE : FieldConstants.GoalCoordinates.RED;

        goalCoordinatesInitialized = true;
    }

    @Override
    public void initialize() {


        flywheel = new Flywheel(
                ActiveOpMode.hardwareMap().get(DcMotorEx.class, MapSetterConstants.leftFlywheelMotorDeviceName),
                ActiveOpMode.hardwareMap().get(DcMotorEx.class, MapSetterConstants.rightFlywheelMotorDeviceName)
        );

        flywheel.setInternalParameters(
                ConfigurationConstants.FLYWHEEL_ASSEMBLY_TOTAL_WEIGHT,
                ConfigurationConstants.FLYWHEEL_SHAFT_DIAMETER,
                ConfigurationConstants.FLYWHEEL_MOTOR_CORE_VOLTAGE,
                ConfigurationConstants.FLYWHEEL_MOTOR_RPM
        );

        flywheel.initVoltageSensor(ActiveOpMode.hardwareMap());
        flywheel.setVelocityPIDVSCoefficients(ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS);

        flywheel.reset();

        hood = new Hood(new HoodAngler(ActiveOpMode.hardwareMap(), MapSetterConstants.hoodAnglerServoDeviceName));

        hood.setFlywheelVelocityAdjustmentParameters(
                ShooterConstants.FLYWHEEL_VELOCITY_HOOD_CORRECTION_INFLUENCE,
                ShooterConstants.FLYWHEEL_VELOCITY_HOOD_CORRECTION_MINIMUM,
                ShooterConstants.FLYWHEEL_VELOCITY_HOOD_CORRECTION_MAXIMUM
        );
        hood.provideFlywheel(flywheel);

        hood.accessHoodAngler().setSafePosition(ShooterConstants.HOOD_ANGLER_MAX_POSITION);

        turret = new TurretBase(ActiveOpMode.hardwareMap());

        turret.setVelocityCoefficients(ConfigurationConstants.TURRET_VELOCITY_COEFFICIENTS);
        turret.setPositionalCoefficients(ConfigurationConstants.TURRET_POSITIONAL_COEFFICIENTS);

        //turret.reverse();
    }

    public Command setVel(Pose robotPoseAtShoot) {

        Pose goalCoordinate;
        if (robotPoseAtShoot.getY() > ShooterConstants.FAR_ZONE_CLOSE_ZONE_BARRIER) {
            goalCoordinate = goalCoordinates.getCloseCoordinate(robotPoseAtShoot.getX(), goalCoordinates);
        }
        else {
            goalCoordinate = goalCoordinates.getFarCoordinate();
        }

        double vel = Shooter.getFlywheelTargetVelocityFromInterpolation(robotPoseAtShoot, new PoseVelocity(), goalCoordinate);
        return new InstantCommand(() -> flywheel.setVelocity(vel, false));
    }

    public Command setVel(double vel) {
        return new InstantCommand(() -> flywheel.setVelocity(vel, false));
    }

    public void end() {
        flywheel.setVelocity(0, true);
    }

    private boolean start = false;
    public void startShooter() {
        start = true;
    }

    @Override
    public void periodic() {

        if ((!start) || (!followerInitialized) || (!goalCoordinatesInitialized)) return;

        //no auto aim
        if (!autoAim) {
            flywheel.update();
            hood.update(0);
            turret.update();
        }

        //with auto aim
        poseSpeedTracker.update();

        PoseVelocity robotVelocity = poseSpeedTracker.getPoseVelocity();

        Pose currentRobotPose = follower.getPose().minus(new Pose(72, 72));
        double robotHeadingRad = currentRobotPose.getHeading();

        Pose turretPose = Calculations.getTurretPoseFromBotPose(currentRobotPose, turret.getCurrentPosition(), turret.startPosition);

        //hood
        hood.setAimZone(
                currentRobotPose.getY() > ShooterConstants.FAR_ZONE_CLOSE_ZONE_BARRIER
                        ? Hood.AimZone.CLOSE
                        : Hood.AimZone.FAR
        );

        //changing the coordinate that the turret aims at based on targeted zones determined by distance
        Pose goalCoordinate;
        if (currentRobotPose.getY() > ShooterConstants.FAR_ZONE_CLOSE_ZONE_BARRIER) {
            goalCoordinate = goalCoordinates.getCloseCoordinate(currentRobotPose.getX(), goalCoordinates);
        }
        else {
            goalCoordinate = goalCoordinates.getFarCoordinate();
        }

        double distanceToGoal = Calculations.getDistanceFromGoal(turretPose.getX(), turretPose.getY(), goalCoordinate);

        //turret
        double flywheelCurrentVelocity = flywheel.getCurrentVelocity() > ShooterConstants.FLYWHEEL_CONSIDERATION_VELOCITY ? flywheel.getCurrentVelocity() : 0;
        double ballSpeed = Models.getBallSpeedFromFlywheel(flywheelCurrentVelocity);
        double timeOfFlight = flywheelCurrentVelocity != 0 ? distanceToGoal / ballSpeed : 0;

        Pose virtualGoal = Calculations.getVirtualGoalCoordinate(timeOfFlight, robotVelocity, goalCoordinate);

        double distanceToVirtualGoal = Calculations.getDistanceFromGoal(turretPose.getX(), turretPose.getY(), virtualGoal);
        timeOfFlight = flywheelCurrentVelocity != 0 ? distanceToVirtualGoal / ballSpeed : 0;

        virtualGoal = Calculations.getVirtualGoalCoordinate(timeOfFlight, robotVelocity, goalCoordinate);

        double angleToGoal = Calculations.getAngleToGoal(turretPose.getX(), turretPose.getY(), virtualGoal);

        double rawtt = MathUtil.normalizeAngleDeg(Math.toDegrees(robotHeadingRad) - angleToGoal);
        double tt = Calculations.routeTurret(rawtt);

        double turretAimPosition = tt * ShooterConstants.TURRET_TICKS_PER_DEGREE + turret.startPosition;

        turret.setAim(turretAimPosition, robotVelocity);

        //updating
        flywheel.update();
        hood.update(distanceToGoal);
        turret.update();
    }

    private boolean autoAim = true;

    public Command setAimState(boolean autoAim) {

        return new InstantCommand(() -> this.autoAim = autoAim);
    }

    public Command turretToZero() {
        return new InstantCommand(() -> {

            autoAim = false;

            turret.setVelocity(0);
            turret.setPosition(turret.startPosition);
        });
    }
}