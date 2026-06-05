package org.firstinspires.ftc.teamcode.Tuners.DriveTuning;

import static org.firstinspires.ftc.teamcode.Constants.ConfigurationConstants.TURRET_POSITIONAL_COEFFICIENTS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.Calculations;
import org.firstinspires.ftc.teamcode.Constants.CameraConstants;
import org.firstinspires.ftc.teamcode.Constants.ConfigurationConstants;
import org.firstinspires.ftc.teamcode.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Systems.Blocker;
import org.firstinspires.ftc.teamcode.Systems.CurrentAlliance;
import org.firstinspires.ftc.teamcode.Systems.FlywheelPIDVSCoefficients;
import org.firstinspires.ftc.teamcode.Systems.Intake;
import org.firstinspires.ftc.teamcode.Systems.TurretBasePIDFCoefficients;
import org.firstinspires.ftc.teamcode.TeleOp.PostAutonomousRobotReset;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.TeleOp.drive.PedroDrive;
import org.firstinspires.ftc.teamcode.util.MathUtil;

@Config
@TeleOp (group = "tuning")
public class FlywheelTurretDriveTuning extends TeleOpBaseOpMode {

    public CurrentAlliance.ALLIANCE ALLIANCE = CurrentAlliance.ALLIANCE.RED_ALLIANCE;

    public static class FlywheelTuning {

        public double KP_FAR = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.kpFar;
        public double KP_CLOSE = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.kpClose;
        public double KI_FAR = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.kiFar;
        public double KI_CLOSE = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.kiClose;
        public double KD = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.kd;
        public double KV = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.unscaledKv;
        public double KS = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.ks;
        public double KPIDF_UNITS_PER_VOLT = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.kPIDFUnitsPerVolt;
        public double I_SWITCH = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.iSwitch;
        public double P_SWITCH = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.pSwitch;
        public double KI_SMASH = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.kISmash;
        public double VOLTAGE_COMPENSATION_WEIGHT = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.voltageCompensationWeight;
        public double VOLTAGE_FILTER_ALPHA = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.voltageFilterAlpha;
        public double TUNING_VOLTAGE = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.tuningVoltage;

        public double D_MIN = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.minD, D_MAX = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.maxD;
        public double I_MIN = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.minI, I_MAX = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.maxI;
        public double P_MIN = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.minP, P_MAX = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS.maxP;
    }

    public static class TurretTuning {

        public double KP_FAR = TURRET_POSITIONAL_COEFFICIENTS.kpFar;
        public double KP_CLOSE = TURRET_POSITIONAL_COEFFICIENTS.kpClose;
        public double[] KI_FAR = {TURRET_POSITIONAL_COEFFICIENTS.lkiFar, TURRET_POSITIONAL_COEFFICIENTS.rkiFar};
        public double[] KI_CLOSE = {TURRET_POSITIONAL_COEFFICIENTS.lkiClose, TURRET_POSITIONAL_COEFFICIENTS.rkiClose};
        public double KD_FAR = TURRET_POSITIONAL_COEFFICIENTS.kdFar;
        public double KD_CLOSE = TURRET_POSITIONAL_COEFFICIENTS.kdClose;
        public double KHOLD = TURRET_POSITIONAL_COEFFICIENTS.unscaledKHold;

        public double P_SWITCH = TURRET_POSITIONAL_COEFFICIENTS.pSwitch;

        public double[] I_SWITCH = TURRET_POSITIONAL_COEFFICIENTS.iSwitch;

        public double D_SWITCH = TURRET_POSITIONAL_COEFFICIENTS.dSwitch;

        public double[] KI_SMASH = {TURRET_POSITIONAL_COEFFICIENTS.lkISmash, TURRET_POSITIONAL_COEFFICIENTS.rkISmash};

        public double KD_FILTER = TURRET_POSITIONAL_COEFFICIENTS.kDFilter;
        public double KVELOCITY_FILTER = TURRET_POSITIONAL_COEFFICIENTS.kVelocityFilter;

        double[] D_ACTIVATION = TURRET_POSITIONAL_COEFFICIENTS.dActivation;

        public double HOLD_DECAY = TURRET_POSITIONAL_COEFFICIENTS.holdDecay;
        public double TUNING_VOLTAGE = TURRET_POSITIONAL_COEFFICIENTS.tuningVoltage;
        public double[] VOLTAGE_FILTER_ALPHA = TURRET_POSITIONAL_COEFFICIENTS.voltageFilterAlpha;

        public double MIN_I = TURRET_POSITIONAL_COEFFICIENTS.minI, MAX_I = TURRET_POSITIONAL_COEFFICIENTS.maxI;
    }

    public static FlywheelTuning FLYWHEEL = new FlywheelTuning();
    public static TurretTuning TURRET = new TurretTuning();

    public static double FLYWHEEL_VELOCITY = 0;
    public static double HOOD_POSITION = 1;

    private Telemetry telemetry;

    private final FlywheelPIDVSCoefficients flywheelCoefficients = ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS;

    private final Intake intake = new Intake();
    private final Blocker blocker = new Blocker().asSubsystem();

    private final PedroDrive pedroDrive = new PedroDrive();

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        initializeDevices();

        applyComponentTraits();

        blocker.provideComponents(super.blocker, controller1);
        intake.provideComponents(super.intake, dropDown, blocker, controller1);
        pedroDrive.provideComponents(follower, controller1);

        //setup lynx module
        setUpLynxModule();
    }

    private FieldConstants.GoalCoordinates goalCoordinates;

    @Override
    public void start() {

        flywheel.setVelocityPIDVSCoefficients(flywheelCoefficients);

        camera.start();

        goalCoordinates = ALLIANCE == CurrentAlliance.ALLIANCE.BLUE_ALLIANCE ? FieldConstants.GoalCoordinates.BLUE : FieldConstants.GoalCoordinates.RED;

        camera.pipelineSwitch(CameraConstants.PIPELINES.GENERAL_GOAL_PIPELINE.getPipelineIndex());

        camera.start();

        flywheel.reset();

        new PostAutonomousRobotReset(this);
    }

    @Override
    public void loop() {

        clearCacheOfLynxModule();

        controller1.getInformation();

        flywheelCoefficients.updateCoefficients(
                FLYWHEEL.KP_FAR, FLYWHEEL.KP_CLOSE,
                FLYWHEEL.KI_FAR, FLYWHEEL.KI_CLOSE,
                FLYWHEEL.KD,
                FLYWHEEL.KV,
                FLYWHEEL.KS,
                FLYWHEEL.KPIDF_UNITS_PER_VOLT,
                FLYWHEEL.I_SWITCH,
                FLYWHEEL.P_SWITCH,
                FLYWHEEL.KI_SMASH,
                FLYWHEEL.VOLTAGE_COMPENSATION_WEIGHT,
                FLYWHEEL.VOLTAGE_FILTER_ALPHA,
                FLYWHEEL.TUNING_VOLTAGE,
                FLYWHEEL.P_MIN, FLYWHEEL.P_MAX,
                FLYWHEEL.I_MIN, FLYWHEEL.I_MAX,
                FLYWHEEL.D_MIN, FLYWHEEL.D_MAX
        );

        flywheel.setVelocityPIDVSCoefficients(flywheelCoefficients);

        hoodAngler.setSafePosition(HOOD_POSITION);
        intake.update();
        blocker.update();
        follower.update();
        turret();
        flywheel.setVelocity(FLYWHEEL_VELOCITY, true);
        flywheel.update();
        pedroDrive.update();

        telemetry.addData("flywheel target velocity", flywheel.getTargetVelocity());
        telemetry.addData("flywheel current velocity", flywheel.getCurrentVelocity());
        telemetry.addData("flywheel p", "%.5f", flywheel.p);
        telemetry.addData("flywheel i", "%.5f", flywheel.i);
        telemetry.addData("flywheel d", "%.5f", flywheel.d);
        telemetry.addData("flywheel v", "%.5f", flywheel.v);

        telemetry.addData("flywheel power", "%.5f", flywheel.getPower());

        telemetry.addData("flywheel is kp far being used", flywheel.getError() > FLYWHEEL.P_SWITCH);
        telemetry.addData("flywheel is kp far being used (graphics)", flywheel.getError() > FLYWHEEL.P_SWITCH ? 1 : 0);

        telemetry.addData("turret target position", turret.getTargetPosition());
        telemetry.addData("turret current position", turret.getCurrentPosition());
        telemetry.addData("turret position error", turret.getError());
        telemetry.addData("turret target velocity", turret.getActuatorTargetVelocity());
        telemetry.addData("turret current velocity", turret.getActuator().getCurrentVelocity());

        telemetry.addData("turret p", "%.5f", turret.p);
        telemetry.addData("turret i", "%.5f", turret.i);
        telemetry.addData("turret d", "%.5f", turret.d);
        telemetry.addData("turret f", "%.5f", turret.f);

        telemetry.addData("turret actuator p", "%.5f", turret.getActuator().p);
        telemetry.addData("turret actuator i", "%.5f", turret.getActuator().i);
        telemetry.addData("turret actuator d", "%.5f", turret.getActuator().d);
        telemetry.addData("turret actuator v", "%.5f", turret.getActuator().v);

        telemetry.addData("turret power", "%.5f", turret.getPower());

        telemetry.update();
    }

    private double filteredVoltageRecord = 0;

    public void turret() {

        Pose robotPose = follower.getPose();

        Pose turretPose = Calculations.getTurretPoseFromBotPose(robotPose, turret.getCurrentPosition(), turret.startPosition);

        //changing the coordinate that the turret aims at based on targeted zones determined by distance
        Pose goalCoordinate;
        if (robotPose.getY() > ShooterConstants.FAR_ZONE_CLOSE_ZONE_BARRIER) {
            goalCoordinate = goalCoordinates.getCloseCoordinate(robotPose.getY(), goalCoordinates);
        }
        else {
            goalCoordinate = goalCoordinates.getFarCoordinate();
        }

        double angleToGoal = Calculations.getAngleToGoal(turretPose.getX(), turretPose.getY(), goalCoordinate);

        double rawtt = MathUtil.normalizeAngleDeg(Math.toDegrees(robotPose.getHeading()) - angleToGoal);
        double tt = Calculations.routeTurret(rawtt);

        turret.setPosition(tt * ShooterConstants.TURRET_TICKS_PER_DEGREE + turret.startPosition);

        turret.setPositionalCoefficients(new TurretBasePIDFCoefficients(
                TURRET.KP_FAR,
                TURRET.KP_CLOSE,
                TURRET.KI_FAR,
                TURRET.KI_CLOSE,
                TURRET.KD_FAR,
                TURRET.KD_CLOSE,
                TURRET.KHOLD,
                TURRET.P_SWITCH,
                TURRET.I_SWITCH,
                TURRET.D_SWITCH,
                TURRET.KI_SMASH,
                TURRET.D_ACTIVATION,
                TURRET.KD_FILTER,
                TURRET.KVELOCITY_FILTER,
                TURRET.HOLD_DECAY,
                TURRET.TUNING_VOLTAGE,
                TURRET.VOLTAGE_FILTER_ALPHA,
                TURRET.MIN_I, TURRET.MAX_I
        ));
        turret.setTuning(true);
        turret.coefficients.filteredVoltage = filteredVoltageRecord;
        turret.update();
        filteredVoltageRecord = turret.coefficients.filteredVoltage;
    }

}