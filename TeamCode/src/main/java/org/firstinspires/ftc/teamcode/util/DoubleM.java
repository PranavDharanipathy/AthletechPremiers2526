package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

public class DoubleM {

    private double value;

    public DoubleM(double value) {
        this.value = value;
    }

    public DoubleM(@NonNull DoubleM valueObj) {
        this.value = valueObj.get();
    }

    public void set(double value) {
        this.value = value;
    }

    public void set(@NonNull DoubleM valueObj) {
        this.value = valueObj.get();
    }

    public double get() {
        return value;
    }

    public DoubleM getDoubleM() {
        return this;
    }

    //---simple operations---

    public void add(double value) {
        this.value += value;
    }

    public void add(@NonNull DoubleM valueObj) {
        this.value += valueObj.get();
    }

    public void subtract(double value) {
        this.value -= value;
    }

    public void subtract(@NonNull DoubleM valueObj) {
        this.value -= valueObj.get();
    }

    public void multiply(double value) {
        this.value *= value;
    }

    public void multiply(@NonNull DoubleM valueObj) {
        this.value *= valueObj.get();
    }

    public void divide(double value) {

        if (value == 0) throw new ArithmeticException("Error divide by zero!");

        this.value = value;
    }

    public void divide(@NonNull DoubleM valueObj) {

        if (valueObj.get() == 0) throw new ArithmeticException("Error divide by zero!");


        this.value = valueObj.get();
    }
}
