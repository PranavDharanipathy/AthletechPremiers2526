package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Systems.Blocker;
import org.firstinspires.ftc.teamcode.Systems.CurrentAlliance;
import org.firstinspires.ftc.teamcode.Systems.Intake;
import org.firstinspires.ftc.teamcode.Systems.Shooter;
import org.firstinspires.ftc.teamcode.TeleOp.PostAutonomousRobotReset;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.TeleOp.drive.PedroDrive;

@Config
@TeleOp(group = "tuning")
public class ShootWhileMovingTuner extends TeleOpBaseOpMode { //turret hysteresis control

    public static CurrentAlliance.ALLIANCE ALLIANCE = CurrentAlliance.ALLIANCE.BLUE_ALLIANCE;

    public static double NOMINAL_DT = 0.06; //dt in seconds
    public static double SHOOTING_TIME = 0.08; //time it takes for ball to leave the shooter once contacting the flywheel
    public static double MAX_VELOCITY = 0.08; //maximum velocity in rad/s
    public static double MAX_ACCEL = 0.08; //maximum velocity is rad/s^2

    public static double POSITIONAL_DIFFERENCE_SCALING = 1;
    public static double ADJUSTMENT_POTENTIAL_SCALING = 1;
    public static double DIRECTION_FORCE_SCALING = 1;

    private final Intake intake = new Intake();
    private final Blocker blocker = new Blocker().asSubsystem();
    private final PedroDrive pedroDrive = new PedroDrive();
    private final Shooter shooter = new Shooter();

    private Telemetry telemetry;

    @Override
    public void init() {

        initializeDevices();
        applyComponentTraits();


        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(30);

        intake.provideComponents(super.intake, transfer, controller1);
        blocker.provideComponents(super.blocker, controller1);
        pedroDrive.provideComponents(follower, ALLIANCE, controller1, controller2);
        shooter.provideComponents(flywheel, turret, hoodAngler, follower, camera, controller1, controller2);
        setUpLynxModule();

        shooter.setTHCTuning(true);
    }

    @Override
    public void start() {

        new PostAutonomousRobotReset(this);

        shooter.start(CurrentAlliance.ALLIANCE.BLUE_ALLIANCE);
    }

    @Override
    public void loop() {

        clearCacheOfLynxModule();

        controller1.getInformation();
        controller2.getInformation();

        intake.update();
        follower.update();
        shooter.provideCustomTHCTime(this::calculation); //shooter applies turret acceleration automatically
        shooter.update();
        blocker.update();
        pedroDrive.update();

        telemetry.addData("turret lookahead time", shooter.getTHCLookahead());
        telemetry.addData("robot current pose", shooter.currentRobotPose);
        telemetry.addData("robot future pose", shooter.futureRobotPose);
        telemetry.update();
    }

    public double calculation(double turretCurrentPosition, double turretPositionError) {

        double errorRad = Math.abs(Math.toRadians(turretPositionError / ShooterConstants.TURRET_TICKS_PER_DEGREE));

        return
                POSITIONAL_DIFFERENCE_SCALING * (errorRad / MAX_VELOCITY) +
                        ADJUSTMENT_POTENTIAL_SCALING * (MAX_VELOCITY / MAX_ACCEL) +
                        DIRECTION_FORCE_SCALING * ((turretCurrentPosition + Math.signum(turretPositionError)) / MAX_ACCEL) +
                        SHOOTING_TIME +
                        NOMINAL_DT;
    }

}
