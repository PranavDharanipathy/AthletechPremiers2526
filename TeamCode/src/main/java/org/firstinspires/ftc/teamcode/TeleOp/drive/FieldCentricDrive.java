package org.firstinspires.ftc.teamcode.TeleOp.drive;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.Subsystem;

public class FieldCentricDrive extends Subsystem {

    private DcMotor left_front, right_front, left_back, right_back;

    private Follower follower;

    private BetterGamepad controller1;

    public void provideComponents(DcMotor left_front, DcMotor right_front, DcMotor left_back, DcMotor right_back, Follower follower, BetterGamepad controller1) {

        this.left_front = left_front;
        this.right_front = right_front;
        this.left_back = left_back;
        this.right_back = right_back;

        this.follower = follower;

        this.controller1 = controller1;
    }

    private double startHeading = 0;

    @Override
    public void update() {

        double botHeading = follower.getHeading();

        double y = -controller1.left_stick_y(); // reversed
        double x = controller1.left_stick_x();
        double rx = controller1.right_stick_x();

        if (controller1.aHasJustBeenPressed) startHeading = botHeading;

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(startHeading - botHeading) - y * Math.sin(startHeading - botHeading);
        double rotY = x * Math.sin(startHeading - botHeading) + y * Math.cos(startHeading - botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        left_front.setPower(frontLeftPower);
        left_back.setPower(backLeftPower);
        right_front.setPower(frontRightPower);
        right_back.setPower(backRightPower);
    }
}