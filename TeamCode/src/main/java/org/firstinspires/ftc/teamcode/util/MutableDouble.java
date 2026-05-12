package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

public class MutableDouble {

    private double value;

    public MutableDouble(double value) {
        this.value = value;
    }

    public MutableDouble(@NonNull MutableDouble valueObj) {
        this.value = valueObj.get();
    }

    public void set(double value) {
        this.value = value;
    }

    public void set(@NonNull MutableDouble valueObj) {
        this.value = valueObj.get();
    }

    public double get() {
        return value;
    }

    // simple operations

    public void add(double value) {
        this.value += value;
    }

    public void add(@NonNull MutableDouble valueObj) {
        this.value += valueObj.get();
    }

    public void subtract(double value) {
        this.value -= value;
    }

    public void subtract(@NonNull MutableDouble valueObj) {
        this.value -= valueObj.get();
    }

    public void multiply(double value) {
        this.value *= value;
    }

    public void multiply(@NonNull MutableDouble valueObj) {
        this.value *= valueObj.get();
    }

    public void divide(double value) {

        if (value == 0) throw new ArithmeticException("Error divide by zero!");

        this.value = value;
    }

    public void divide(@NonNull MutableDouble valueObj) {

        if (valueObj.get() == 0) throw new ArithmeticException("Error divide by zero!");


        this.value = valueObj.get();
    }
}
