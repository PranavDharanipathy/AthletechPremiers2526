package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.Blocker;
import org.firstinspires.ftc.teamcode.Systems.CurrentAlliance;
import org.firstinspires.ftc.teamcode.Systems.Intake;
import org.firstinspires.ftc.teamcode.Systems.Lift;
import org.firstinspires.ftc.teamcode.Systems.Shooter;
import org.firstinspires.ftc.teamcode.TeleOp.drive.PedroDrive;

@TeleOp(name = "TeleOp RED", group = "Match")
public class TeleOp_RED extends TeleOpBaseOpMode {

    private final CurrentAlliance alliance = new CurrentAlliance(CurrentAlliance.ALLIANCE.RED_ALLIANCE);

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

        pedroDrive.provideComponents(follower, alliance.getAlliance(), controller1, controller2);
        intake.provideComponents(super.intake, transfer, controller1);
        blocker.provideComponents(super.blocker, controller1);
        tiltLift.provideComponents(lift, controller2);
        shooter.provideComponents(flywheel, turret, hoodAngler, follower, camera, controller1, controller2);
        telemetry.provideComponents(super.telemetry, true, controller2);
        setUpLynxModule();
    }

    @Override
    public void start() {

        new PostAutonomousRobotReset(this);

        shooter.start(alliance.getAlliance());
    }

    @Override
    public void loop() {

        clearCacheOfLynxModule();

        controller1.getInformation();
        controller2.getInformation();

        intake.update();
        follower.update();
        shooter.update();
        blocker.update();
        tiltLift.update();
        pedroDrive.update();

        telemetry.runInstance(shooter, pedroDrive);
    }

}
