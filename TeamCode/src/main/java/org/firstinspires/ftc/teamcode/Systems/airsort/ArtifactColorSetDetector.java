package org.firstinspires.ftc.teamcode.Systems.airsort;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.util.BetterColorSensor;
import org.firstinspires.ftc.teamcode.util.MathUtil;

public class ArtifactColorSetDetector {

    public static BetterColorSensor color1st; //closest to shooter
    public static BetterColorSensor color2nd; //middle
    public static BetterColorSensor color3rd; //closest to the front of the intake

    public ArtifactColorSetDetector(HardwareMap hardwareMap, String[] colorSensorNames) {

        color1st = new BetterColorSensor(hardwareMap, colorSensorNames[0]);
        color2nd = new BetterColorSensor(hardwareMap, colorSensorNames[1]);
        color3rd = new BetterColorSensor(hardwareMap, colorSensorNames[2]);

        color1st.setGain(IntakeConstants.COLOR_SENSOR_GAINS[0]);
        color2nd.setGain(IntakeConstants.COLOR_SENSOR_GAINS[1]);
        color3rd.setGain(IntakeConstants.COLOR_SENSOR_GAINS[2]);
    }

    public String[] getBallColorSet() {

        return new String[] {
                getColorAsIdentifier(color1st.getRGB()),
                getColorAsIdentifier(color2nd.getRGB()),
                getColorAsIdentifier(color3rd.getRGB())
        };
    }

    private String getColorAsIdentifier(float[] detectedRGB) {

        if (withinColorThreshold(detectedRGB, GeneralConstants.PURPLE_ARTIFACT_RGB_THRESHOLD)) {
            return IntakeConstants.BallIdentifiers.PURPLE_ARTIFACT;
        }
        else if (withinColorThreshold(detectedRGB, GeneralConstants.GREEN_ARTIFACT_RGB_THRESHOLD)) {
            return IntakeConstants.BallIdentifiers.GREEN_ARTIFACT;
        }
        else {
            return IntakeConstants.BallIdentifiers.NO_ARTIFACT;
        }

    }

    /// @param threshold index 0 is the minimum and index 1 is the maximum
    /// @return whether the detected rgb values fit into the threshold values
    private boolean withinColorThreshold(float[] detected, float[][] threshold) {

        final boolean withinRed = MathUtil.valueWithinRangeIncludingPoles(detected[0], threshold[0][0], threshold[1][0]);
        final boolean withinGreen = MathUtil.valueWithinRangeIncludingPoles(detected[1], threshold[0][1], threshold[1][1]);
        final boolean withinBlue = MathUtil.valueWithinRangeIncludingPoles(detected[2], threshold[0][2], threshold[1][2]);

        return withinRed && withinGreen && withinBlue;
    }
}
