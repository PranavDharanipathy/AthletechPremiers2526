package org.firstinspires.ftc.teamcode.util;

public class BooleanTrigger {

    private boolean trigger;

    public BooleanTrigger(boolean trigger) {
        this.trigger = trigger;
    }

    public void set(boolean trigger) {
        this.trigger = trigger;
    }

    public BooleanTrigger and(BooleanTrigger booleanTrigger) {
        return new BooleanTrigger(booleanTrigger.get() && trigger);
    }

    public BooleanTrigger or(BooleanTrigger booleanTrigger) {
        return new BooleanTrigger(booleanTrigger.get() || trigger);
    }

    public void setAnd(BooleanTrigger booleanTrigger) {
        trigger = booleanTrigger.get() && trigger;
    }

    public void setOr(BooleanTrigger booleanTrigger) {
        trigger = booleanTrigger.get() || trigger;
    }

    public void setNot(BooleanTrigger booleanTrigger) {
        trigger = !booleanTrigger.get();
    }

    public boolean get() {
        return trigger;
    }
}
