package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.Calculations;
import org.firstinspires.ftc.teamcode.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.TeleOp.PostAutonomousRobotReset;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.TeleOp.drive.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.util.TickrateChecker;

@Config
@TeleOp (group = "tuning")
public class HoodRegressionTuner extends TeleOpBaseOpMode {

    public static double TRANSFER_VELOCITY = 1850;
    public static double INTAKE_POWER = 1;
    public static double FLYWHEEL_VELOCITY = 0;
    public static double HOOD_POSITION = 0.3;

    public enum GOAL {

        RED(FieldConstants.GoalCoordinatesForDistance.RED.getCoordinate()), BLUE(FieldConstants.GoalCoordinatesForDistance.BLUE.getCoordinate());

        private Pose coord;

        GOAL(Pose goalCoordinate) {
            coord = goalCoordinate;
        }

        public Pose getCoordinate() {
            return coord;
        }
    }

    public static GOAL goal = GOAL.BLUE;

    private final RobotCentricDrive robotCentricDrive = new RobotCentricDrive();

    private Telemetry telemetry;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(10);

        initializeDevices();

        applyComponentTraits();

        robotCentricDrive.provideComponents(left_front, right_front, left_back, right_back, controller1);

        //setup lynx module
        setUpLynxModule();
    }

    @Override
    public void start() {
        new PostAutonomousRobotReset(this);
    }

    @Override
    public void loop() {

        // clear data at start of loop
        clearCacheOfLynxModule();

        controller1.getInformation();

        hoodAngler.setPosition(HOOD_POSITION);
        intake.setPower(INTAKE_POWER);
        transfer.setVelocity(TRANSFER_VELOCITY);
        flywheel.setVelocity(FLYWHEEL_VELOCITY, true);
        transfer.update();
        flywheel.update();

        if (controller1.main_buttonHasJustBeenPressed) {
            relocalization(FieldConstants.RELOCALIZATION_POSE);
        }

        follower.update();

        Pose robotPose = follower.getPose();
        double robotYawRad = robotPose.getHeading();
        Pose turretPose = Calculations.getTurretPoseFromBotPose(robotPose, 0, 0);

        double distanceToGoal = Calculations.getDistanceFromGoal(turretPose.getX(), turretPose.getY(), goal.getCoordinate());

        robotCentricDrive.update();

        telemetry.addData("Tick rate", TickrateChecker.getTimePerTick());

        telemetry.addData("p", flywheel.p);
        telemetry.addData("i", flywheel.i);
        telemetry.addData("d", flywheel.d);
        telemetry.addData("v", flywheel.v);

        telemetry.addData("flywheel current velocity", flywheel.getCurrentVelocity());
        telemetry.addData("flywheel target velocity", flywheel.getTargetVelocity());

        telemetry.addData("turret current position", turret.getCurrentPosition());
        telemetry.addData("turret target position", turret.getTargetPosition());
        telemetry.addData("turret position error", turret.getPositionError());

        telemetry.addData("turret position + robot heading", "%.2f, %.2f, %.2f", turretPose.getX(), turretPose.getY(), Math.toDegrees(robotYawRad));
        telemetry.addData("distance to goal", distanceToGoal);
        telemetry.addData("robot pose", robotPose);

        telemetry.addData("goal coordinate", "x:%.2f, y:%.2f", goal.getCoordinate().getX(), goal.getCoordinate().getY());

        telemetry.update();

    }


    private void relocalization(Pose pose) {
        follower.setPose(pose);
    }

}
