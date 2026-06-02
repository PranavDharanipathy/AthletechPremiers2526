package org.firstinspires.ftc.teamcode.Systems;

import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.Subsystem;

public class Intake extends Subsystem {

    private IntakeActuator intakeSystem;

    private Blocker blockerSubsystem;

    private BetterGamepad controller1;

    public void provideComponents(IntakeActuator intake, Blocker blockerSubsystem, BetterGamepad controller1) {

        intakeSystem = intake;

        this.blockerSubsystem = blockerSubsystem;

        this.controller1 = controller1;
    }

    @Override
    public void update() {

        if (blockerSubsystem.getState() == Blocker.BlockerState.CLEAR) {
            intakeSystem.setIntakePower(IntakeConstants.INTAKE_POWER);
            intakeSystem.setTransferVelocity(IntakeConstants.TRANSFER_VELOCITY);
        }
        else if (controller1.right_trigger(GeneralConstants.TRIGGER_THRESHOLD)) {
            intakeSystem.setIntakePower(IntakeConstants.INTAKE_POWER);
            intakeSystem.setTransferVelocity(IntakeConstants.TRANSFER_IDLE_VELOCITY);
        }
        else if (controller1.left_trigger(GeneralConstants.TRIGGER_THRESHOLD)) {
            intakeSystem.setIntakePower(IntakeConstants.REVERSE_INTAKE_POWER);
            intakeSystem.setTransferPower(IntakeConstants.REVERSE_TRANSFER_POWER);
        }
        else {
            intakeSystem.setIntakePower(0);
            intakeSystem.setTransferVelocity(0);
        }

        intakeSystem.update();
    }
}