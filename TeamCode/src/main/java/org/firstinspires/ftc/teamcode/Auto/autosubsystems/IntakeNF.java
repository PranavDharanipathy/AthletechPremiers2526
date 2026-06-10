package org.firstinspires.ftc.teamcode.Auto.autosubsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.ConfigurationConstants;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Constants.MapSetterConstants;
import org.firstinspires.ftc.teamcode.Systems.Blocker;
import org.firstinspires.ftc.teamcode.Systems.IntakeActuator;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.ServoEx;


public class IntakeNF implements Subsystem {

    private IntakeNF() {}

    public static final IntakeNF INSTANCE = new IntakeNF();

    public IntakeActuator intake;

    public Blocker blocker;

    public ServoEx dropDown;

    @Override
    public void initialize() {

        intake = new IntakeActuator(ActiveOpMode.hardwareMap());
        intake.setVelocityPDFCoefficients(
                ConfigurationConstants.TRANSFER_PDF_COEFFICIENTS[0],
                ConfigurationConstants.TRANSFER_PDF_COEFFICIENTS[1],
                ConfigurationConstants.TRANSFER_PDF_COEFFICIENTS[2]
        );

        blocker = new Blocker(ActiveOpMode.hardwareMap().get(Servo.class, MapSetterConstants.blockerServoDeviceName));
        blocker.setState(Blocker.BlockerState.BLOCK);

        dropDown = new ServoEx(MapSetterConstants.intakeDropDownServoDeviceName);
    }

    private double[] transferVelocity = {0, IntakeConstants.CLOSE_TRANSFER_VELOCITY};

    public Command custom(double intakePower, double transferVelocity) {

        return new InstantCommand(() -> {
            dropDown.setPosition(IntakeConstants.DROPDOWN_INTAKING_POSITION);
            intake.setIntakePower(intakePower);
            this.transferVelocity[0] = transferVelocity;
        });
    }

    public Command intake() {

        return new InstantCommand(() -> {
            dropDown.setPosition(IntakeConstants.DROPDOWN_INTAKING_POSITION);
            intake.setIntakePower(IntakeConstants.INTAKE_POWER);
            transferVelocity[0] = IntakeConstants.TRANSFER_IDLE_VELOCITY;
        });
    }

    public Command reverse() {

        return new InstantCommand(() -> {
            dropDown.setPosition(IntakeConstants.DROPDOWN_INTAKING_POSITION);
            intake.setIntakePower(IntakeConstants.REVERSE_INTAKE_POWER);
            transferVelocity[0] = -IntakeConstants.CLOSE_TRANSFER_VELOCITY;
        });
    }

    public Command idle() {
        return new InstantCommand(() -> dropDown.setPosition(IntakeConstants.DROPDOWN_IDLE_POSITION));
    }

    public Command blocker(boolean shoot) {

        Runnable blockerCmd = shoot ? () -> blocker.setState(Blocker.BlockerState.CLEAR) : () -> blocker.setState(Blocker.BlockerState.BLOCK);

        return new InstantCommand(blockerCmd)
                .and(new InstantCommand(() -> dropDown.setPosition(IntakeConstants.DROPDOWN_INTAKING_POSITION)));
    }

    public Command stop() {

        return new InstantCommand(() -> {
            dropDown.setPosition(IntakeConstants.DROPDOWN_IDLE_POSITION);
            intake.setIntakePower(0);
            transferVelocity[0] = 0;
        });
    }

    public void end() {
        intake.setIntakePower(0);
        intake.setTransferVelocity(0);
    }

    @Override
    public void periodic() {

        intake.setTransferVelocity(blocker.getState() == Blocker.BlockerState.CLEAR ? transferVelocity[1] : transferVelocity[0]);

        intake.update();
    }
}