package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.Blocker;
import org.firstinspires.ftc.teamcode.Systems.CurrentAlliance;
import org.firstinspires.ftc.teamcode.Systems.Intake;
import org.firstinspires.ftc.teamcode.Systems.Lift;
import org.firstinspires.ftc.teamcode.Systems.Shooter;
import org.firstinspires.ftc.teamcode.TeleOp.drive.PedroDrive;

@Config
@TeleOp(name = "TeleOp TEST", group = "testing")
public class TeleOp_TEST extends TeleOpBaseOpMode {

    public static CurrentAlliance.ALLIANCE ALLIANCE = CurrentAlliance.ALLIANCE.BLUE_ALLIANCE;

    private final Intake intake = new Intake();
    private final Blocker blocker = new Blocker().asSubsystem();
    private final Lift tiltLift = new Lift().asSubsystem();
    private final Shooter shooter = new Shooter();
    private final PedroDrive pedroDrive = new PedroDrive();
    private final TelemetrySubsystem telemetry = new TelemetrySubsystem();

    @Override
    public void init() {

        initializeDevices();
        applyComponentTraits();

        pedroDrive.provideComponents(follower, controller1);
        intake.provideComponents(super.intake, blocker /*subsystem*/, transfer, controller1);
        blocker.provideComponents(super.blocker, controller1);
        tiltLift.provideComponents(lift, controller2);
        shooter.provideComponents(flywheel, turret, hoodAngler, follower, camera, controller1, controller2);
        telemetry.provideComponents(super.telemetry, true, controller2);
        setUpLynxModule();
    }

    @Override
    public void start() {

        new PostAutonomousRobotReset(this);

        shooter.start(ALLIANCE);
    }

    @Override
    public void loop() {

        clearCacheOfLynxModule();

        controller1.getInformation();
        controller2.getInformation();

        shooter.switchAlliance(ALLIANCE);

        intake.update();
        follower.update();
        shooter.update();
        blocker.update();
        tiltLift.update();
        pedroDrive.update();

        telemetry.runInstance(shooter, pedroDrive);
    }

}