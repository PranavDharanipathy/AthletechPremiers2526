package org.firstinspires.ftc.teamcode.Tuners.PoseKalmanFilter;

import static org.firstinspires.ftc.teamcode.Tuners.PoseKalmanFilter.PoseKalmanFilterProcessNoiseStdDevTuningDashboard.PROCESS_NOISE_STD_DEVS;
import static org.firstinspires.ftc.teamcode.Tuners.PoseKalmanFilter.PoseKalmanFilterStandardDeviationTuner.follower;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.LocalizationConstants;
import org.firstinspires.ftc.teamcode.Constants.MapSetterConstants;
import org.firstinspires.ftc.teamcode.Systems.Camera;
import org.firstinspires.ftc.teamcode.Systems.PoseEstimator;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.PedroPathing.PoseSpeedTracker;
import org.firstinspires.ftc.teamcode.util.StandardDeviationCalculator;

import java.util.ArrayList;
import java.util.List;

@TeleOp(group = "tuning")
public class PoseKalmanFilterStandardDeviationTuner extends SelectableOpMode {

    public static Follower follower;

    public static Telemetry telemetry;

    public PoseKalmanFilterStandardDeviationTuner() {
        super("Select a Tuning OpMode", l -> {
            l.add("Odometry", Odometry::new);
            l.add("Camera", Vision::new);
            l.add("Process Noise", ProcessNoise::new);
            l.add("Test", PoseKalmanFilterTest::new);
        });
    }

    @Override
    public void onSelect() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        if (follower == null) {
            follower = LocalizationConstants.createFollower(hardwareMap);
        } else {
            follower = LocalizationConstants.createFollower(hardwareMap);
        }

        follower.setPose(new Pose());
    }

    @Override
    public void onLog(List<String> lines) {}
}

class PoseKalmanFilterTest extends OpMode {

    private PoseEstimator poseEstimator;

    private Camera camera;

    private BetterGamepad controller1;

    @Override
    public void init() {

        controller1 = new BetterGamepad(gamepad1);

        camera = new Camera(follower, Camera.from(hardwareMap, MapSetterConstants.limelight3AUSBDeviceName));

        poseEstimator = new PoseEstimator(
                new Pose(),
                LocalizationConstants.ODOMETRY_STD_DEV,
                LocalizationConstants.CAMERA_STD_DEV,
                LocalizationConstants.PROCESS_NOISE_STD_DEV
        );
    }

    @Override
    public void loop() {

        controller1.getInformation();

        follower.update();
        camera.update(controller1.main_buttonHasJustBeenPressed, poseEstimator::reset);
        poseEstimator.update(follower, camera);

        telemetry.addData("Odometry Pose", follower.getPose());
        telemetry.addData("Camera Pose", camera.getBotPoseMT2());
        telemetry.addData("KF Pose", poseEstimator.getPose());

        telemetry.update();
    }
}

class Odometry extends OpMode {

    private final double TOTAL_TIME = 60d;
    private final double DT = 70d;

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
        timestamp = 0;
    }

    private double timestamp;

    @Override
    public void loop() {

        double millis = timer.milliseconds();

        if ((millis * 1000d) <= TOTAL_TIME && timer.milliseconds() - timestamp >= DT) {

            follower.update();

            Pose odoPose = follower.getPose();

            xData.add(odoPose.getX());
            yData.add(odoPose.getY());
            thetaData.add(odoPose.getHeading());

            timestamp = millis;
        }

        if (timer.seconds() > TOTAL_TIME) {

            telemetry.addData("x std dev", StandardDeviationCalculator.getSample(xData.stream().mapToDouble(Double::doubleValue).toArray()));
            telemetry.addData("y std dev", StandardDeviationCalculator.getSample(yData.stream().mapToDouble(Double::doubleValue).toArray()));
            telemetry.addData("theta std dev", StandardDeviationCalculator.getSample(thetaData.stream().mapToDouble(Double::doubleValue).toArray()));
        }

        telemetry.addData("Pose", follower.getPose());

        telemetry.addData("Time remaining", TOTAL_TIME - (millis * 1000d));

        telemetry.update();

    }
}

class Vision extends OpMode {

    private final double TOTAL_TIME = 60d;
    private final double DT = 70d;

    private Camera camera;

    private BetterGamepad controller1;

    private ElapsedTime timer = new ElapsedTime();

    private List<Double> xData = new ArrayList<>();
    private List<Double> yData = new ArrayList<>();
    private List<Double> thetaData = new ArrayList<>();

    @Override
    public void init() {

        camera = new Camera(follower, Camera.from(hardwareMap, MapSetterConstants.limelight3AUSBDeviceName));

        controller1 = new BetterGamepad(gamepad1);
    }

    @Override
    public void start() {

        follower.setPose(new Pose());
        follower.update();

        timer.reset();
        timestamp = 0;
    }

    private double timestamp;

    @Override
    public void loop() {

        controller1.getInformation();

        if (!camera.isEligibleForMT2()) timer.reset();

        double millis = timer.milliseconds();

        if ((millis * 1000d) <= TOTAL_TIME && timer.milliseconds() - timestamp >= DT) {

            follower.update();
            camera.update(controller1.main_buttonHasJustBeenPressed);

            if (camera.canUseMT2Pose()) {

                Pose cameraPose = camera.getBotPoseMT2();

                xData.add(cameraPose.getX());
                yData.add(cameraPose.getY());
                thetaData.add(cameraPose.getHeading());

                timestamp = millis;
            }
        }

        if (timer.seconds() > TOTAL_TIME) {

            telemetry.addData("x std dev", StandardDeviationCalculator.getSample(xData.stream().mapToDouble(Double::doubleValue).toArray()));
            telemetry.addData("y std dev", StandardDeviationCalculator.getSample(yData.stream().mapToDouble(Double::doubleValue).toArray()));
            telemetry.addData("theta std dev", StandardDeviationCalculator.getSample(thetaData.stream().mapToDouble(Double::doubleValue).toArray()));
        }

        telemetry.addData("Pose", camera.getBotPoseMT2());

        telemetry.addData("Time remaining", TOTAL_TIME - (millis * 1000d));

        telemetry.update();

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

    @Override
    public void init() {

        controller1 = new BetterGamepad(gamepad1);

        camera = new Camera(follower, Camera.from(hardwareMap, MapSetterConstants.limelight3AUSBDeviceName));

        poseSpeedTracker = new PoseSpeedTracker(follower);

        poseEstimator = new PoseEstimator(
                new Pose(),
                LocalizationConstants.ODOMETRY_STD_DEV,
                LocalizationConstants.CAMERA_STD_DEV,
                PROCESS_NOISE_STD_DEVS
        );
    }

    @Override
    public void start() {
        follower.startTeleOpDrive(true);
    }

    @Override
    public void loop() {

        controller1.getInformation();

        poseEstimator.setProcessNoiseStdDev(PROCESS_NOISE_STD_DEVS);

        follower.setTeleOpDrive(
                -controller1.left_stick_y(),
                controller1.left_stick_x(),
                controller1.right_stick_x()
        );

        follower.update();
        poseSpeedTracker.update();

        camera.update(controller1.main_buttonHasJustBeenPressed, poseEstimator::reset);
        poseEstimator.update(follower, camera);

        //pose
        telemetry.addData("Odo X", follower.getPose().getX());
        telemetry.addData("Odo Y", follower.getPose().getY());
        telemetry.addData("Odo Theta", follower.getPose().getHeading());

        telemetry.addData("Camera X", camera.getBotPoseMT2().getX());
        telemetry.addData("Camera Y", camera.getBotPoseMT2().getY());
        telemetry.addData("Camera Theta", camera.getBotPoseMT2().getHeading());

        telemetry.addData("KF X", poseEstimator.getPose().getX());
        telemetry.addData("KF Y", poseEstimator.getPose().getY());
        telemetry.addData("KF Theta", poseEstimator.getPose().getHeading());

        //vel
        telemetry.addData("Odo X Vel", poseSpeedTracker.getPoseVelocity().getXVelocity());
        telemetry.addData("Odo Y Vel", poseSpeedTracker.getPoseVelocity().getYVelocity());
        telemetry.addData("Odo Theta Vel", poseSpeedTracker.getPoseVelocity().getAngularVelocity());

        telemetry.addData("KF X Vel", poseEstimator.getPoseVelocity().getXVelocity());
        telemetry.addData("KF Y Vel", poseEstimator.getPoseVelocity().getYVelocity());
        telemetry.addData("KF Theta Vel", poseEstimator.getPoseVelocity().getAngularVelocity());

        telemetry.update();
    }
}