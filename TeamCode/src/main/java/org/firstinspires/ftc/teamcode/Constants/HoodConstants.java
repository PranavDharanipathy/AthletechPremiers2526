package org.firstinspires.ftc.teamcode.Constants;


import java.util.ArrayList;
import java.util.List;

public class HoodConstants {

    /// Index 0 is minimum voltage outputted and index 1 is maximum voltage outputted.
    public static double[] HOOD_ANALOG_ENCODER_VOLTAGE_RANGE = {0, 3.3};

    public static List<Double> DISTANCES = new ArrayList<>();
    public static List<Double> HOOD_POSITIONS = new ArrayList<>();

    public static double MINIMUM_FLYWHEEL_VELOCITY_HOOD_CORRECTION = 0.004;
    public static double FLYWHEEL_VELOCITY_HOOD_CORRECTION_DEADBAND = 0.001;
}