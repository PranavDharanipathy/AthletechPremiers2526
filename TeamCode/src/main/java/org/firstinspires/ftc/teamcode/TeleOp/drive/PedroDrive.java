package org.firstinspires.ftc.teamcode.TeleOp.drive;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.HeadingInterpolator;

import org.firstinspires.ftc.teamcode.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;
import org.firstinspires.ftc.teamcode.Systems.CurrentAlliance;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.FastSigmoidCurve;
import org.firstinspires.ftc.teamcode.util.Subsystem;

public class PedroDrive extends Subsystem {

    private Follower follower;
    public Follower getFollower() {
        return follower;
    }

    private BetterGamepad controller1;

    public void provideComponents(Follower follower, BetterGamepad controller1) {

        this.follower = follower;

        this.controller1 = controller1;
    }

    private final FastSigmoidCurve curve = new FastSigmoidCurve();

    @Override
    public void update() {

        if (!follower.getTeleopDrive()) {
            follower.breakFollowing();
            follower.startTeleOpDrive(true);
        }

        double forward = deadbandJoystick(-controller1.left_stick_y());
        double strafe = deadbandJoystick(controller1.left_stick_x());
        double rotation = deadbandJoystick(controller1.right_stick_x());

        double forwardSigmoid = Math.signum(forward) * curve.getOutput(Math.abs(forward));
        double strafeSigmoid = Math.signum(strafe) * curve.getOutput(Math.abs(strafe));
        double rotationSigmoid = Math.signum(rotation) * curve.getOutput(Math.abs(rotation));

        follower.setTeleOpDrive(
                forwardSigmoid,
                strafeSigmoid,
                rotationSigmoid
        );

    }

    private double deadbandJoystick(double value) {
        return Math.abs(value) > GeneralConstants.JOYSTICK_MINIMUM ? value : 0;
    }

}