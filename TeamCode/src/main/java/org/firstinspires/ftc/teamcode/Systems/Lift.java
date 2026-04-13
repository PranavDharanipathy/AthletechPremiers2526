package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.ConfigurationConstants;
import org.firstinspires.ftc.teamcode.Constants.LiftConstants;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.Subsystem;

public class Lift extends Subsystem {

    private boolean isSubsystem;

    private Object lift; //given Lift (non-subsystem) or Servo data

    private BetterGamepad controller2;

    public Lift(Servo lift) {

        isSubsystem = false;

        lift.setDirection(ConfigurationConstants.LIFT_SERVO_DIRECTION);
        this.lift = lift;
    }
    public Lift() {}

    public Lift asSubsystem() {
        isSubsystem = true;
        return this;
    }

    // COMPONENT
    public enum LiftState {
        RETRACTED(LiftConstants.LIFT_RETRACTED_POSITION), LIFT(LiftConstants.LIFT_LIFT_POSITION);

        private double position;

        LiftState(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    private LiftState state;

    public void setState(LiftState state) {

        if (this.state == state) return;

        this.state = state;

        if (isSubsystem) {

            ((Lift) lift).setState(state);
        }
        else {
            ((Servo) lift).setPosition(this.state.getPosition());
        }
    }

    public LiftState getState() {
        return state;
    }

    // SUBSYSTEM
    public void provideComponents(Lift lift, BetterGamepad controller2) {

        this.lift = lift;

        this.controller2 = controller2;
    }

    private boolean liftToggle = false;

    @Override
    public void update() {

        if (controller2.right_triggerHasJustBeenPressed) {

            if (liftToggle) setState(LiftState.RETRACTED);
            else setState(LiftState.LIFT);

            liftToggle = !liftToggle;
        }

    }
}