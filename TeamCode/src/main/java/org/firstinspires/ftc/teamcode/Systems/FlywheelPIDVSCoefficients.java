package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Constants.Models;
import org.firstinspires.ftc.teamcode.util.DoubleM;
import org.firstinspires.ftc.teamcode.util.LowPassFilter;

/// Easier usage of the coefficients for the flywheel
public class FlywheelPIDVSCoefficients {

    public double kpFar, kpClose;
    public double kiFar, kiClose;
    public double kd;
    public double unscaledKv;
    public double ks;

    public double kPIDFUnitsPerVolt;

    public double iSwitch;
    public double pSwitch;

    public double kISmash;

    public double voltageFilterAlpha;

    public double minP, maxP;
    public double minI, maxI;
    public double minD, maxD;

    private boolean tuning = false;

    public void setTuning(boolean tuning) {
        this.tuning = tuning;
    }

    public FlywheelPIDVSCoefficients(
            double kpFar,
            double kpClose,
            double kiFar,
            double kiClose,
            double kd,
            double unscaledKv,
            double ks,
            double kPIDFUnitsPerVolt,
            double iSwitch,
            double pSwitch,
            double kISmash,
            double voltageFilterAlpha,
            double minP,
            double maxP,
            double minI,
            double maxI,
            double minD,
            double maxD
    ) {

        this.kpFar = kpFar;
        this.kpClose = kpClose;

        this.kiFar = kiFar;
        this.kiClose = kiClose;

        this.kd = kd;

        this.unscaledKv = unscaledKv;

        this.ks = ks;
        this.kPIDFUnitsPerVolt = kPIDFUnitsPerVolt;

        this.iSwitch = iSwitch;
        this.pSwitch = pSwitch;

        this.kISmash = kISmash;

        this.voltageFilterAlpha = voltageFilterAlpha;

        this.minP = minP;
        this.maxP = maxP;

        this.minI = minI;
        this.maxI = maxI;

        this.minD = minD;
        this.maxD = maxD;
    }

    public void updateCoefficients(
            double kpFar,
            double kpClose,
            double kiFar,
            double kiClose,
            double kd,
            double unscaledKv,
            double ks,
            double kPIDFUnitsPerVolt,
            double iSwitch,
            double pSwitch,
            double kISmash,
            double voltageFilterAlpha,
            double minP,
            double maxP,
            double minI,
            double maxI,
            double minD,
            double maxD
    ) {

        this.kpFar = kpFar;
        this.kpClose = kpClose;

        this.kiFar = kiFar;
        this.kiClose = kiClose;

        this.kd = kd;

        this.unscaledKv = unscaledKv;

        this.ks = ks;
        this.kPIDFUnitsPerVolt = kPIDFUnitsPerVolt;

        this.iSwitch = iSwitch;
        this.pSwitch = pSwitch;

        this.kISmash = kISmash;

        this.voltageFilterAlpha = voltageFilterAlpha;

        this.minP = minP;
        this.maxP = maxP;

        this.minI = minI;
        this.maxI = maxI;

        this.minD = minD;
        this.maxD = maxD;
    }

    public double kp(double targetVelocity, double currentVelocity) {

        if (Math.abs(targetVelocity - currentVelocity) < pSwitch) return kpClose;
        else return kpFar;
    }

    private double kISwitchTargetVelocity;

    public double ki(double targetVelocity, double currentVelocity, DoubleM errorSum) {

        if (kISwitchTargetVelocity == targetVelocity || Math.abs(targetVelocity - currentVelocity) < iSwitch) {

            if (kISwitchTargetVelocity != targetVelocity) errorSum.set(0);

            kISwitchTargetVelocity = targetVelocity;
            return kiClose;
        }
        else return kiFar;
    }

    private double filteredVoltage = 0;

    /// Run every loop
    public double kv(VoltageSensor batteryVoltageSensor) {

        if (tuning) return unscaledKv;

        filteredVoltage = LowPassFilter.getFilteredValue(filteredVoltage, batteryVoltageSensor.getVoltage(), voltageFilterAlpha);

        return Models.getScaledFlywheelKv(unscaledKv, filteredVoltage);
    }
}