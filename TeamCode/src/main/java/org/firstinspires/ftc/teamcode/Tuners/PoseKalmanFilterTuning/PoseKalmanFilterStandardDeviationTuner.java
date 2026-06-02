package org.firstinspires.ftc.teamcode.Tuners.PoseKalmanFilterTuning;

import static org.firstinspires.ftc.teamcode.Tuners.PoseKalmanFilterTuning.PoseKalmanFilterStandardDeviationTuner.follower;

import static org.firstinspires.ftc.teamcode.Tuners.PoseKalmanFilterTuning.PoseKalmanFilterProcessNoiseStdDevTuningDashboard.X;
import static org.firstinspires.ftc.teamcode.Tuners.PoseKalmanFilterTuning.PoseKalmanFilterProcessNoiseStdDevTuningDashboard.Y;
import static org.firstinspires.ftc.teamcode.Tuners.PoseKalmanFilterTuning.PoseKalmanFilterProcessNoiseStdDevTuningDashboard.THETA;
import static org.firstinspires.ftc.teamcode.Tuners.PoseKalmanFilterTuning.PoseKalmanFilterProcessNoiseStdDevTuningDashboard.VX;
import static org.firstinspires.ftc.teamcode.Tuners.PoseKalmanFilterTuning.PoseKalmanFilterProcessNoiseStdDevTuningDashboard.VY;
import static org.firstinspires.ftc.teamcode.Tuners.PoseKalmanFilterTuning.PoseKalmanFilterProcessNoiseStdDevTuningDashboard.VTHETA;
import static org.firstinspires.ftc.teamcode.Tuners.PoseKalmanFilterTuning.PoseKalmanFilterStandardDeviationTuner.telemetryM;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.CameraConstants;
import org.firstinspires.ftc.teamcode.Constants.LocalizationConstants;
import org.firstinspires.ftc.teamcode.Constants.MapSetterConstants;
import org.firstinspires.ftc.teamcode.Systems.Camera;
import org.firstinspires.ftc.teamcode.Systems.PoseEstimator;
import org.firstinspires.ftc.teamcode.TeleOp.drive.PedroDrive;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.PedroPathing.PoseSpeedTracker;
import org.firstinspires.ftc.teamcode.util.StandardDeviationCalculator;

import java.util.ArrayList;
import java.util.List;

@Configurable
@TeleOp(group = "tuning")
public class PoseKalmanFilterStandardDeviationTuner extends SelectableOpMode {

    public static Follower follower;

    public static TelemetryManager telemetryM;

    public PoseKalmanFilterStandardDeviationTuner() {
        super("Select a Tuning OpMode", s -> {
            s.folder("Localization", l -> {
                l.add("Odometry", Odometry::new);
                l.add("Camera", Vision::new);
                l.add("Process Noise", ProcessNoise::new);
                l.add("Test", PoseKalmanFilterTest::new);
            });
        });
    }

    @Override
    public void onSelect() {

        if (follower == null) {
            follower = LocalizationConstants.createFollower(hardwareMap);
        } else {
            follower = LocalizationConstants.createFollower(hardwareMap);
        }

        follower.setPose(new Pose());

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void onLog(List<String> lines) {}
}

class PoseKalmanFilterTest extends OpMode {

    private PoseEstimator poseEstimator;

    private Camera camera;

    private PedroDrive drive = new PedroDrive();

    private BetterGamepad controller1;

    @Override
    public void init() {

        controller1 = new BetterGamepad(gamepad1);

        drive.provideComponents(follower, controller1);

        camera = new Camera(follower, Camera.from(hardwareMap, MapSetterConstants.limelight3AUSBDeviceName));
        camera.pipelineSwitch(CameraConstants.PIPELINES.GENERAL_GOAL_PIPELINE.getPipelineIndex());

        poseEstimator = new PoseEstimator(
                new Pose(),
                LocalizationConstants.ODOMETRY_STD_DEV,
                LocalizationConstants.CAMERA_STD_DEV,
                LocalizationConstants.PROCESS_NOISE_STD_DEV
        );
    }

    @Override
    public void start() {
        camera.start();
    }

    @Override
    public void loop() {

        controller1.getInformation();

        follower.update();
        camera.update(controller1.main_buttonHasJustBeenPressed, poseEstimator::reset);
        poseEstimator.update(follower, camera);

        drive.update();

        telemetryM.debug("Odometry Pose", follower.getPose());
        telemetryM.debug("Camera Pose", camera.getBotPoseMT2());
        telemetryM.debug("KF Pose", poseEstimator.getPose());

        telemetryM.update(telemetry);
    }
}

class Odometry extends OpMode {

    private final double TOTAL_TIME = 60d;

    private ElapsedTime timer = new ElapsedTime();

    private List<Double> xData = new ArrayList<>();
    private List<Double> yData = new ArrayList<>();
    private List<Double> thetaData = new ArrayList<>();

    @Override
    public void init() {}

    @Override
    public void start() {

        follower.setPose(new Pose());
        follower.update();

        timer.reset();
    }

    @Override
    public void loop() {

        if (timer.seconds() <= TOTAL_TIME) {

            follower.update();

            Pose odoPose = follower.getPose();

            xData.add(odoPose.getX());
            yData.add(odoPose.getY());
            thetaData.add(odoPose.getHeading());
        }

        if (timer.seconds() > TOTAL_TIME) {

            telemetryM.debug("x std dev" + StandardDeviationCalculator.getSample(xData.stream().mapToDouble(Double::doubleValue).toArray()));
            telemetryM.debug("y std dev" + StandardDeviationCalculator.getSample(yData.stream().mapToDouble(Double::doubleValue).toArray()));
            telemetryM.debug("theta std dev" + StandardDeviationCalculator.getSample(thetaData.stream().mapToDouble(Double::doubleValue).toArray()));
        }

        telemetryM.debug("Pose" + follower.getPose());

        telemetryM.debug("Time remaining" + (TOTAL_TIME - timer.seconds()));

        telemetryM.update(telemetry);

    }
}

class Vision extends OpMode {

    private final double TOTAL_TIME = 60d;

    private Camera camera;

    private BetterGamepad controller1;

    private ElapsedTime timer = new ElapsedTime();

    private List<Double> xData = new ArrayList<>();
    private List<Double> yData = new ArrayList<>();
    private List<Double> thetaData = new ArrayList<>();

    @Override
    public void init() {

        camera = new Camera(follower, Camera.from(hardwareMap, MapSetterConstants.limelight3AUSBDeviceName));
        camera.pipelineSwitch(CameraConstants.PIPELINES.GENERAL_GOAL_PIPELINE.getPipelineIndex());

        controller1 = new BetterGamepad(gamepad1);
    }

    @Override
    public void start() {

        follower.setPose(new Pose(0,0,Math.toRadians(90)));
        follower.update();

        camera.start();

        timer.reset();
    }

    @Override
    public void loop() {

        controller1.getInformation();

        follower.update();
        camera.update(controller1.main_buttonHasJustBeenPressed);

        if (!camera.isEligibleForMT2()) timer.reset();

        if (timer.seconds() <= TOTAL_TIME) {

            if (camera.canUseMT2Pose()) {

                Pose cameraPose = camera.getBotPoseMT2();

                xData.add(cameraPose.getX());
                yData.add(cameraPose.getY());
                thetaData.add(cameraPose.getHeading());
            }
        }

        if (timer.seconds() > TOTAL_TIME) {

            telemetryM.debug("x std dev" + StandardDeviationCalculator.getSample(xData.stream().mapToDouble(Double::doubleValue).toArray()));
            telemetryM.debug("y std dev", + StandardDeviationCalculator.getSample(yData.stream().mapToDouble(Double::doubleValue).toArray()));
            telemetryM.debug("theta std dev", + StandardDeviationCalculator.getSample(thetaData.stream().mapToDouble(Double::doubleValue).toArray()));
        }

        telemetryM.debug("Pose" + camera.getBotPoseMT2());

        telemetryM.debug("Time remaining", + (TOTAL_TIME - timer.seconds()));

        telemetryM.update(telemetry);

    }
}

/**
 * <h3>TUNING GUIDE</h3>
 * Larger value => filter trusts measurements more, predictions less
 * Smaller value => filter trusts predictions more, measurements less
 * <p>
 * Estimate lags behind? => increase value
 * Estimate is jittery/laggy? => decrease value/undo change
 */
class ProcessNoise extends OpMode {

    private PoseEstimator poseEstimator;

    private PoseSpeedTracker poseSpeedTracker;

    private Camera camera;

    private BetterGamepad controller1;

    private PedroDrive drive = new PedroDrive();

    @Override
    public void init() {

        controller1 = new BetterGamepad(gamepad1);

        drive.provideComponents(follower, controller1);

        camera = new Camera(follower, Camera.from(hardwareMap, MapSetterConstants.limelight3AUSBDeviceName));
        camera.pipelineSwitch(CameraConstants.PIPELINES.GENERAL_GOAL_PIPELINE.getPipelineIndex());

        poseSpeedTracker = new PoseSpeedTracker(follower);

        poseEstimator = new PoseEstimator(
                new Pose(),
                LocalizationConstants.ODOMETRY_STD_DEV,
                LocalizationConstants.CAMERA_STD_DEV,
                new double[] {X, Y, THETA, VX, VY, VTHETA}
        );
    }

    @Override
    public void start() {

        follower.startTeleOpDrive(true);

        camera.start();
    }

    private boolean localizedWithCamera = false;

    @Override
    public void loop() {

        controller1.getInformation();

        poseEstimator.setProcessNoiseStdDev(new double[] {X, Y, THETA, VX, VY, VTHETA});

        follower.setTeleOpDrive(
                -controller1.left_stick_y(),
                controller1.left_stick_x(),
                controller1.right_stick_x()
        );

        follower.update();
        poseSpeedTracker.update();

        if (!localizedWithCamera) {
            localizedWithCamera = controller1.main_buttonHasJustBeenPressed;
        }

        camera.update(controller1.main_buttonHasJustBeenPressed, poseEstimator::reset);
        poseEstimator.update(follower, camera);

        drive.update();

        //pose
        telemetryM.debug("Odo X: " + follower.getPose().getX());
        telemetryM.debug("Odo Y: " + follower.getPose().getY());
        telemetryM.debug("Odo Theta: " + follower.getPose().getHeading());

        telemetryM.debug("Localized with camera: " + localizedWithCamera);

        if (localizedWithCamera && camera.getBotPoseMT2() != null) {
            telemetryM.debug("Camera X: " + camera.getBotPoseMT2().getX());
            telemetryM.debug("Camera Y: " + camera.getBotPoseMT2().getY());
            telemetryM.debug("Camera Theta: " + camera.getBotPoseMT2().getHeading());
        }

        telemetryM.debug("KF X: " + poseEstimator.getPose().getX());
        telemetryM.debug("KF Y: " + poseEstimator.getPose().getY());
        telemetryM.debug("KF Theta: " + poseEstimator.getPose().getHeading());

        //vel
        telemetryM.debug("Odo X Vel: " + poseSpeedTracker.getPoseVelocity().getXVelocity());
        telemetryM.debug("Odo Y Vel: " + poseSpeedTracker.getPoseVelocity().getYVelocity());
        telemetryM.debug("Odo Theta Vel: " + poseSpeedTracker.getPoseVelocity().getAngularVelocity());

        telemetryM.debug("KF X Vel: " + poseEstimator.getPoseVelocity().getXVelocity());
        telemetryM.debug("KF Y Vel: " + poseEstimator.getPoseVelocity().getYVelocity());
        telemetryM.debug("KF Theta Vel: " + poseEstimator.getPoseVelocity().getAngularVelocity());

        telemetryM.update(telemetry);
    }
}