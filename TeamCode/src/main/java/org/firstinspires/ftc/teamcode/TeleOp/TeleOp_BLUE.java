package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.Blocker;
import org.firstinspires.ftc.teamcode.Systems.CurrentAlliance;
import org.firstinspires.ftc.teamcode.Systems.Flywheel;
import org.firstinspires.ftc.teamcode.Systems.Intake;
import org.firstinspires.ftc.teamcode.Systems.Lift;
import org.firstinspires.ftc.teamcode.Systems.Shooter;
import org.firstinspires.ftc.teamcode.TeleOp.drive.PedroDrive;

@TeleOp(name = "TeleOp BLUE", group = "A_Match")
public class TeleOp_BLUE extends TeleOpBaseOpMode {

    private final CurrentAlliance alliance = new CurrentAlliance(CurrentAlliance.ALLIANCE.BLUE_ALLIANCE);

    private final Intake intake = new Intake();
    private final Blocker blocker = new Blocker().asSubsystem();
    private final PedroDrive pedroDrive = new PedroDrive();
    private final TelemetrySubsystem telemetry = new TelemetrySubsystem();

    @Override
    public void init() {

        initializeDevices();
        applyComponentTraits();

        pedroDrive.provideComponents(follower, controller1);
        intake.provideComponents(super.intake, blocker /*subsystem*/, transfer, controller1);
        blocker.provideComponents(super.blocker, controller1);
        telemetry.provideComponents(super.telemetry);
        setUpLynxModule();
    }

    @Override
    public void start() {

        new PostAutonomousRobotReset(this);

    }

    @Override
    public void loop() {

        clearCacheOfLynxModule();

        controller1.getInformation();
        //controller2.getInformation();

        intake.update();
        follower.update();
        shooter();
        blocker.update();
        pedroDrive.update();

        telemetry.runInstance();
    }

    private void shooter() {

        flywheel.runMotor(Flywheel.RunningMotor.DISABLE);
        flywheel.setPower(-1);

        flywheel.update();
    }
}