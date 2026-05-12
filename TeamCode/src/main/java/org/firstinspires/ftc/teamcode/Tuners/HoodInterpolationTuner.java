package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.Calculations;
import org.firstinspires.ftc.teamcode.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.Systems.Hood;
import org.firstinspires.ftc.teamcode.TeleOp.PostAutonomousRobotReset;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.TeleOp.drive.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.util.TickrateChecker;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp (group = "tuning")
public class HoodInterpolationTuner extends TeleOpBaseOpMode {

    public static double TRANSFER_VELOCITY = 1800;
    public static double INTAKE_POWER = 1;
    public static double FLYWHEEL_VELOCITY = 0;

    public static double HOOD_POSITION = 0.3;

    public static double FV_INFLUENCE = 0;

    public static double FV_CORRECTION_MIN = -1;
    public static double FV_CORRECTION_MAX = 1;

    private final List<Double> distances = new ArrayList<>();
    private final List<Double> positions = new ArrayList<>();

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

    private Hood hood;
    private final RobotCentricDrive robotCentricDrive = new RobotCentricDrive();

    private Telemetry telemetry;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(10);

        initializeDevices();

        applyComponentTraits();
        //camera.setAutomaticOdometryRelocalization(false);

        hood = new Hood(hoodAngler);
        robotCentricDrive.provideComponents(left_front, right_front, left_back, right_back, controller1);

        //setup lynx module
        setUpLynxModule();
    }

    @Override
    public void start() {
        new PostAutonomousRobotReset(this);
    }

    private boolean seeHoodInterpolationDataset = false;

    @Override
    public void loop() {

        // clear data at start of loop
        clearCacheOfLynxModule();

        controller1.getInformation();

        if (controller1.yHasJustBeenPressed) {
            seeHoodInterpolationDataset = !seeHoodInterpolationDataset;
        }

        hood.setFlywheelVelocityAdjustmentParameters(FV_INFLUENCE, FV_CORRECTION_MIN, FV_CORRECTION_MAX);

        intake.setPower(INTAKE_POWER);
        transfer.setVelocity(TRANSFER_VELOCITY);
        flywheel.setVelocity(FLYWHEEL_VELOCITY, true);

        transfer.update();
        flywheel.update();

        follower.update();

        Pose robotPose = follower.getPose();
        double robotHeading = robotPose.getHeading();
        Pose turretPose = Calculations.getTurretPoseFromBotPose(robotPose, 0, 0);

        double distanceToGoal = Calculations.getDistanceFromGoal(turretPose.getX(), turretPose.getY(), goal.getCoordinate());

        if (controller1.dpad_upHasJustBeenPressed) {
            distances.add(distanceToGoal);
            positions.add(HOOD_POSITION);
        }
        else if (controller1.dpad_downHasJustBeenPressed && !distances.isEmpty()) {

            int lastIndex = distances.size() - 1;

            distances.remove(distances.get(lastIndex));
            positions.remove(positions.get(lastIndex));
        }

        hood.tuningUpdate(distanceToGoal, distances, positions);

        camera.update(controller1.main_buttonHasJustBeenPressed);

        robotCentricDrive.update();

        telemetry.addLine("<dpad up> to add data point.");
        telemetry.addLine("<dpad down> to remove data point.");
        telemetry.addLine("<Y> to see hood interpolation dataset.");

        if (seeHoodInterpolationDataset) {
            displayHoodInterpolationDataset();
        }
        else {
            telemetry.addLine();

            telemetry.addData("distance to goal", distanceToGoal);
            telemetry.addData("hood position", hood.accessHoodAngler().getPosition());
            telemetry.addData("velocity corrective distance", hood.getCorrectiveDirection());

            telemetry.addLine();

            telemetry.addData("Tick rate", TickrateChecker.getTimePerTick());

            telemetry.addData("turret position + robot heading", "%.2f, %.2f, %.2f", turretPose.getX(), turretPose.getY(), Math.toDegrees(robotHeading));
            telemetry.addData("robot pose", robotPose);
            telemetry.addData("goal coordinate", "x:%.2f, y:%.2f", goal.getCoordinate().getX(), goal.getCoordinate().getY());

            telemetry.addLine();

            telemetry.addData("flywheel current velocity", flywheel.getCurrentVelocity());
            telemetry.addData("flywheel target velocity", flywheel.getTargetVelocity());

            telemetry.addData("turret current position", turret.getCurrentPosition());
            telemetry.addData("turret target position", turret.getTargetPosition());
            telemetry.addData("turret position error", turret.getPositionError());

            telemetry.addData("p", flywheel.p);
            telemetry.addData("i", flywheel.i);
            telemetry.addData("d", flywheel.d);
            telemetry.addData("v", flywheel.v);
        }

        telemetry.update();

    }

    private void displayHoodInterpolationDataset() {

    }

}
