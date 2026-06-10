package org.firstinspires.ftc.teamcode.Systems;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.Subsystem;

public class Intake extends Subsystem {

    private IntakeActuator intakeSystem;

    private Servo dropDown;

    private Blocker blockerSubsystem;

    private Follower follower;

    private BetterGamepad controller1;

    public void provideComponents(IntakeActuator intake, Servo dropDown, Blocker blockerSubsystem, Follower follower, BetterGamepad controller1) {

        intakeSystem = intake;

        this.dropDown = dropDown;

        this.blockerSubsystem = blockerSubsystem;

        this.follower = follower;

        this.controller1 = controller1;
    }

    @Override
    public void update() {

        if (blockerSubsystem.getState() == Blocker.BlockerState.CLEAR) {

            intakeSystem.setIntakePower(IntakeConstants.INTAKE_POWER);

            boolean isClose = follower.getPose().getY() > ShooterConstants.FAR_ZONE_CLOSE_ZONE_BARRIER;
            intakeSystem.setTransferVelocity(isClose ? IntakeConstants.CLOSE_TRANSFER_VELOCITY : IntakeConstants.FAR_TRANSFER_VELOCITY);
        }
        else if (controller1.right_trigger(GeneralConstants.TRIGGER_THRESHOLD)) {
            intakeSystem.setIntakePower(IntakeConstants.INTAKE_POWER);
            intakeSystem.setTransferVelocity(IntakeConstants.TRANSFER_IDLE_VELOCITY);
            dropDown.setPosition(IntakeConstants.DROPDOWN_INTAKING_POSITION);
        }
        else if (controller1.left_trigger(GeneralConstants.TRIGGER_THRESHOLD)) {
            intakeSystem.setIntakePower(IntakeConstants.REVERSE_INTAKE_POWER);
            intakeSystem.setTransferPower(IntakeConstants.REVERSE_TRANSFER_POWER);
            dropDown.setPosition(IntakeConstants.DROPDOWN_INTAKING_POSITION);
        }
        else {
            intakeSystem.setIntakePower(0);
            intakeSystem.setTransferVelocity(0);
            dropDown.setPosition(IntakeConstants.DROPDOWN_IDLE_POSITION);
        }

        intakeSystem.update();
    }
}