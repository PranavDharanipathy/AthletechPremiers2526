package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.ConfigurationConstants;
import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.Subsystem;

public class IntakeDropDown extends Subsystem {

    private boolean isSubsystem;

    private Object dropDown; //given intake drop down (non-subsystem) or Servo data

    private BetterGamepad controller1;

    public IntakeDropDown(Servo dropDown) {

        isSubsystem = false;

        dropDown.setDirection(ConfigurationConstants.INTAKE_DROPDOWN_SERVO_DIRECTION);
        this.dropDown = dropDown;
    }
    public IntakeDropDown() {}

    public IntakeDropDown asSubsystem() {
        isSubsystem = true;
        return this;
    }

    // COMPONENT
    public enum DropDownState {
        INTAKING(IntakeConstants.DROPDOWN_INTAKING_POSITION), IDLE(IntakeConstants.DROPDOWN_IDLE_POSITION);

        private double position;

        DropDownState(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    private DropDownState state;

    public void setState(DropDownState state) {

        if (this.state == state) return;

        this.state = state;

        if (isSubsystem) {

            ((IntakeDropDown) dropDown).setState(state);
        }
        else {
            ((Servo) dropDown).setPosition(this.state.getPosition());
        }
    }

    public DropDownState getState() {
        return state;
    }

    // SUBSYSTEM
    public void provideComponents(IntakeDropDown dropDown, BetterGamepad controller1) {

        this.dropDown = dropDown;

        this.controller1 = controller1;
    }

    @Override
    public void update() {

        if (controller1.right_trigger(GeneralConstants.TRIGGER_THRESHOLD)) {
            setState(DropDownState.INTAKING);
        }
        else {
            setState(DropDownState.IDLE);
        }
    }
}