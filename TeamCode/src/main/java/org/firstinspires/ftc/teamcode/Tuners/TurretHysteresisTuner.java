package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Systems.CurrentAlliance;
import org.firstinspires.ftc.teamcode.Systems.Shooter;
import org.firstinspires.ftc.teamcode.TeleOp.PostAutonomousRobotReset;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.TeleOp.drive.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.util.MathUtil;

@Config
@TeleOp(group = "tuning")
public class TurretHysteresisTuner extends TeleOpBaseOpMode {

    public static CurrentAlliance.ALLIANCE ALLIANCE = CurrentAlliance.ALLIANCE.BLUE_ALLIANCE;

    public static double A_NORMAL = 7.2;
    public static double T_NORMAL = 1.5;
    public static double T_MAX = 2.5;

    private RobotCentricDrive robotCentricDrive = new RobotCentricDrive();
    private Shooter shooter = new Shooter();

    private Telemetry telemetry;

    @Override
    public void init() {

        initializeDevices();
        applyComponentTraits();


        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        robotCentricDrive.provideComponents(left_front, right_front, left_back, right_back, controller1);
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

        shooter.provideCustomTHCLookahead(this::getTurretFuturePosePredictionTime); // shooter applies turret acceleration automatically

        shooter.update();

        telemetry.addData("Turret lookahead time", shooter.getTHCLookahead());
        telemetry.update();
    }


    private double getTurretFuturePosePredictionTime(double turretAcceleration) {

        final double NORMA_ACCEL = Math.toRadians(A_NORMAL);
        final double NORMAL = T_NORMAL;
        final double MAX = T_MAX;

        final double turretAccel = Math.abs(turretAcceleration);

        double futurePredictionTime = NORMAL * (1 + Math.sqrt(NORMA_ACCEL / (turretAccel + 1e-3)));

        return MathUtil.clamp(futurePredictionTime, NORMAL, MAX);
    }

}
