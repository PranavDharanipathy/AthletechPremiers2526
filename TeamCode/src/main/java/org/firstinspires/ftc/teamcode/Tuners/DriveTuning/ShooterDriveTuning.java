package org.firstinspires.ftc.teamcode.Tuners.DriveTuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.MapSetterConstants;
import org.firstinspires.ftc.teamcode.Systems.Blocker;
import org.firstinspires.ftc.teamcode.TeleOp.PostAutonomousRobotReset;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.util.TickrateChecker;

@Config
@TeleOp (group = "tuning")
public class ShooterDriveTuning extends TeleOpBaseOpMode {

    //shooter tuning
    public static double TRANSFER_VELOCITY = 2000;
    public static double FLYWHEEL_VELOCITY = 405_000;
    public static double HOOD_POSITION = 0.2;

    public static boolean SHOOT = false;

    private Blocker blocker;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        blocker = new Blocker(hardwareMap.get(Servo.class, MapSetterConstants.blockerServoDeviceName));

        initializeDevices();

        applyComponentTraits();

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

        hoodAngler.setPosition(HOOD_POSITION);
        blocker.setState(SHOOT ? Blocker.BlockerState.CLEAR : Blocker.BlockerState.BLOCK);
        intake.setTransferVelocity(TRANSFER_VELOCITY);
        intake.update();
        flywheel.setVelocity(FLYWHEEL_VELOCITY, true);
        flywheel.update();

        telemetry.addData("Tick rate", TickrateChecker.getTimePerTick());

        telemetry.addData("hood position", hoodAngler.getPosition());

        telemetry.addData("p", flywheel.p);
        telemetry.addData("i", flywheel.i);
        telemetry.addData("d", flywheel.d);
        telemetry.addData("v", flywheel.v);

        telemetry.addData("flywheel current velocity", flywheel.getCurrentVelocity());
        telemetry.addData("flywheel target velocity", flywheel.getTargetVelocity());

        telemetry.addData("turret current position", turret.getCurrentPosition());
        telemetry.addData("turret target position", turret.getTargetPosition());

        telemetry.update();
    }

}
