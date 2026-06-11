package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.ConfigurationConstants;
import org.firstinspires.ftc.teamcode.Constants.LiftConstants;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.Subsystem;

public class Lift extends Subsystem {

    private CRServo leftLift;
    private CRServo rightLift;

    private BetterGamepad controller1;

    public void provideComponents(CRServo leftLift, CRServo rightLift, BetterGamepad controller1) {

        this.leftLift = leftLift;
        this.rightLift = rightLift;

        this.controller1 = controller1;
    }

    @Override
    public void update() {

        if (controller1.b()) {
            leftLift.setPower(LiftConstants.LIFT_TILT_POWER);
            rightLift.setPower(LiftConstants.LIFT_TILT_POWER);
        }
        else {
            leftLift.setPower(0);
            rightLift.setPower(0);
        }

    }
}