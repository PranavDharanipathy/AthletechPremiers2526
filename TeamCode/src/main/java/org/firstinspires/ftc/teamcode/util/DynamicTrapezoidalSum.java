package org.firstinspires.ftc.teamcode.util;

public class DynamicTrapezoidalSum {

    private double sum;

    private int initialSummationStage;
    private double initialSumAddition;

    public DynamicTrapezoidalSum() {

        sum = 0d;
        initialSummationStage = 1;
    }

    private double lastY;

    public void updateSum(double dx, double y) {

        if (initialSummationStage == 1) {

            initialSumAddition = y * dx;
            sum += initialSumAddition;

            lastY = y;

            initialSummationStage = 2;

            return;
        }

        if (dx != 0 && initialSummationStage == 2) {
            sum -= initialSumAddition;
            initialSummationStage = 0;
        }

        if (dx == 0) {
            sum += y;
        }
        else {
            sum += (dx * y) + (0.5 * dx * (lastY - y)); //adding area of trapezoid
        }

        lastY = y;
    }

    public void setSum(double value) {

        if (sum == value) return;

        sum = value;
        initialSummationStage = 1;
    }

    public void setRawSum(double value) {

        if (sum == value) return;

        sum = value;
    }

    public double getSum() {
        return sum;
    }
}
