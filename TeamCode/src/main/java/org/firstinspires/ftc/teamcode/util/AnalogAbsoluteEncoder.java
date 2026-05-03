package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

/// Absolute Analog mode being used.
public class AnalogAbsoluteEncoder {

    private final AnalogInput encoder;

    public AnalogAbsoluteEncoder(HardwareMap hardwareMap, String deviceName) {

        encoder = hardwareMap.get(AnalogInput.class, deviceName);
    }

    public double range;

    public double minV, maxV;

    public void calibrate(double minV, double maxV, double range) {

        this.minV = minV;
        this.maxV = maxV;

        this.range = range;
    }

    public void calibrate(double minV, double maxV) {
        calibrate(minV, maxV, 360);
    }

    public double getAngle() {

        double voltage = encoder.getVoltage();

        return (voltage - minV) / (maxV - minV) * range;
    }
}