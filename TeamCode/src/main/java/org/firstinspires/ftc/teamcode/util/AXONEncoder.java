package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

/// Absolute Analog mode being used. (simplified)
public class AXONEncoder {

    private final AnalogInput encoder;

    public AXONEncoder(HardwareMap hardwareMap, String deviceName) {

        encoder = hardwareMap.get(AnalogInput.class, deviceName);
    }

    public double getAngle() {
        return (encoder.getVoltage() / 3.3) * 360;
    }
}