package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.GeneralVeloMotor;
import org.firstinspires.ftc.teamcode.util.Subsystem;

public class Intake extends Subsystem {

    private DcMotor intake;
    private GeneralVeloMotor transfer;

    private Blocker blockerSubsystem;

    private BetterGamepad controller1;

    public void provideComponents(DcMotor intake, Blocker blockerSubsystem, GeneralVeloMotor transfer, BetterGamepad controller1) {

        this.intake = intake;
        this.transfer = transfer;

        this.blockerSubsystem = blockerSubsystem;

        this.controller1 = controller1;
    }

    @Override
    public void update() {

        if (controller1.right_trigger(GeneralConstants.JOYSTICK_MINIMUM)) {
            intake.setPower(IntakeConstants.INTAKE_POWER);
            transfer.setVelocity(IntakeConstants.TRANSFER_VELOCITY);
        }
        else if (controller1.left_trigger(GeneralConstants.JOYSTICK_MINIMUM)) {
            intake.setPower(IntakeConstants.REVERSE_INTAKE_POWER);
            transfer.setVelocity(IntakeConstants.REVERSE_TRANSFER_VELOCITY);
        }
        else {
            intake.setPower(0);
            transfer.setVelocity(0);
        }

        if (blockerSubsystem.getState() == Blocker.BlockerState.BLOCK && transfer.getCurrent(CurrentUnit.MILLIAMPS) > IntakeConstants.TRANSFER_CURRENT_LIMIT) {
            transfer.setMotorActivity(GeneralVeloMotor.MotorActivity.STOP);
        }
        else {
            transfer.setMotorActivity(GeneralVeloMotor.MotorActivity.RUN);
        }

        transfer.update();
    }
}