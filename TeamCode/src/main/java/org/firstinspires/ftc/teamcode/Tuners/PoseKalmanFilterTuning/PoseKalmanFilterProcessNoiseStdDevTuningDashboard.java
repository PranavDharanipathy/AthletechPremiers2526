package org.firstinspires.ftc.teamcode.Tuners.PoseKalmanFilterTuning;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.Constants.LocalizationConstants;

@Configurable
public class PoseKalmanFilterProcessNoiseStdDevTuningDashboard {

    public static double X = LocalizationConstants.PROCESS_NOISE_STD_DEV[0];
    public static double Y = LocalizationConstants.PROCESS_NOISE_STD_DEV[1];
    public static double THETA = LocalizationConstants.PROCESS_NOISE_STD_DEV[2];
    public static double VX = LocalizationConstants.PROCESS_NOISE_STD_DEV[3];
    public static double VY = LocalizationConstants.PROCESS_NOISE_STD_DEV[4];
    public static double VTHETA = LocalizationConstants.PROCESS_NOISE_STD_DEV[5];
}
