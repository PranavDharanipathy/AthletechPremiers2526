package org.firstinspires.ftc.teamcode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Systems.Shooter;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.TickrateChecker;
import org.firstinspires.ftc.teamcode.TeleOp.drive.PedroDrive;
import org.firstinspires.ftc.teamcode.util.EffectivelySubsystem;
import org.firstinspires.ftc.teamcode.util.TelemetryUtils.TelemetryMode;
import org.firstinspires.ftc.teamcode.util.TelemetryUtils.TelemetryO;

public class TelemetrySubsystem implements EffectivelySubsystem {

    private TelemetryO telem;

    private BetterGamepad controller2;

    public void provideComponents(Telemetry telemetry, boolean addFTCDashboard, BetterGamepad controller2) {

        telemetry.setMsTransmissionInterval(GeneralConstants.TELEMETRY_MS_TRANSMISSION_INTERVAL);

        telem = new TelemetryO(telemetry, addFTCDashboard);

        telem.setTelemetryModes(TelemetryMode.INFO);

        this.controller2 = controller2;
    }

    public void runInstance(Shooter shooter, PedroDrive pedroDrive) {

        if (controller2.right_bumperHasJustBeenPressed) {

            telem.clearAll();

            if (telem.getTelemetryModes().contains(TelemetryMode.INFO)) {
                telem.setTelemetryModes(TelemetryMode.RAW_DATA);
            }
            else {
                telem.setTelemetryModes(TelemetryMode.INFO);
            }
        }

        telem.addData("Tick rate", TickrateChecker.getTimePerTick());

        telem.addData("LL Localization Outcome", shooter.camera.getMt1LocalizationOutcome());

        telem.addData("ZONE", shooter.getZone().toString());
        telem.addData(TelemetryMode.RAW_DATA, "on alliance side?", shooter.accessGoalCoordinates().onAllianceSide(shooter.futureRobotPose.getY()));

        telem.addData(TelemetryMode.RAW_DATA, "hood position", shooter.hoodAngler.getPosition());

        telem.addData(TelemetryMode.INFO, "flywheel current velocity", "%.0f", shooter.flywheel.getCurrentVelocity());
        telem.addData(TelemetryMode.INFO, "flywheel target velocity", shooter.flywheel.getTargetVelocity());

        telem.addData(TelemetryMode.INFO, "turret target angle", shooter.tt);
        telem.addData(TelemetryMode.INFO, "turret current angle", ((shooter.turret.getTargetPosition() - shooter.turret.startPosition) / ShooterConstants.TURRET_TICKS_PER_DEGREE) - Math.toDegrees(shooter.currentRobotPose.getHeading()));
        telem.addData(TelemetryMode.INFO, "turret position error", shooter.turret.getError());

        telem.addData(TelemetryMode.RAW_DATA, "current robot pose", "x: %.2f, y: %.2f, heading: %.2f", shooter.currentRobotPose.getX(), shooter.currentRobotPose.getY(), Math.toDegrees(shooter.currentRobotPose.getHeading()));
        telem.addData(TelemetryMode.INFO, "future robot pose", "x: %.2f, y: %.2f, heading: %.2f", shooter.futureRobotPose.getX(), shooter.futureRobotPose.getY(), Math.toDegrees(shooter.futureRobotPose.getHeading()));
        telem.addData(TelemetryMode.INFO, "is turret looking ahead", shooter.isTurretLookingAhead());
        telem.addData(TelemetryMode.RAW_DATA, "turret lookahead time", shooter.getTHCLookahead());

        telem.addData(TelemetryMode.RAW_DATA, "f p", "%.5f", shooter.flywheel.p);
        telem.addData(TelemetryMode.RAW_DATA, "f i", "%.5f", shooter.flywheel.i);
        telem.addData(TelemetryMode.RAW_DATA, "f d", "%.5f", shooter.flywheel.d);
        telem.addData(TelemetryMode.RAW_DATA, "f v", "%.5f", shooter.flywheel.v);
        telem.addData(TelemetryMode.RAW_DATA, "flywheel power", shooter.flywheel.getPower());

        telem.addData(TelemetryMode.RAW_DATA, "turret current position", shooter.turret.getCurrentPosition());
        telem.addData(TelemetryMode.RAW_DATA, "turret target position", shooter.turret.getTargetPosition());

        telem.addData(TelemetryMode.RAW_DATA, "t p", "%.5f", shooter.turret.p);
        telem.addData(TelemetryMode.RAW_DATA, "t i", "%.5f", shooter.turret.i);
        telem.addData(TelemetryMode.RAW_DATA, "t d", "%.5f", shooter.turret.d);
        telem.addData(TelemetryMode.RAW_DATA, "t f", "%.5f", shooter.turret.f);
        telem.addData(TelemetryMode.RAW_DATA, "turret power", shooter.turret.getPower());

        telem.addData(TelemetryMode.RAW_DATA, "pedro follower busy?", pedroDrive.getFollower().isBusy());
        telem.addData(TelemetryMode.RAW_DATA, "pedro t value", "%.2f", pedroDrive.getFollower().getCurrentTValue());

        telem.update();
    }
}