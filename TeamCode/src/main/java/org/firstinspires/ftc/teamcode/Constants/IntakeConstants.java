package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class IntakeConstants {

    public static double INTAKE_POWER = 1;
    public static double REVERSE_INTAKE_POWER = -1;

    public static double TRANSFER_VELOCITY = 1800;
    public static double REVERSE_TRANSFER_VELOCITY = -1800;

    /// In milliamps
    public static double TRANSFER_CURRENT_LIMIT = 9000;

    public static class BallIdentifiers {

        public static String PURPLE_ARTIFACT = "P";
        public static String GREEN_ARTIFACT = "G";
        public static String NO_ARTIFACT = "_";
    }

    public static float[] COLOR_SENSOR_GAINS = {2.0f, 2.0f, 2.0f};

}