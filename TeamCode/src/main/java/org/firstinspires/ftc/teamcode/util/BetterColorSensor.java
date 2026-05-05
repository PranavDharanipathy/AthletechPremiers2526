package org.firstinspires.ftc.teamcode.util;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

// w adheesh
public class BetterColorSensor {

    // Sensor and color-related fields
    private final NormalizedColorSensor colorSensor;

    public BetterColorSensor(HardwareMap hardwareMap, String colorSensorName) {

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, colorSensorName);

        // enable light
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }

        setGain(2f); // Set the gain to default
    }

    private float gain;

    public void setGain(float newGain) {

        if (gain == newGain) return;

        gain = newGain;

        colorSensor.setGain(gain);
    }

    public float getGain() {
        return colorSensor.getGain();
    }

    public float[] getRGB() {

        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        return new float[] {
                colors.red,
                colors.green,
                colors.blue
        };
    }

}