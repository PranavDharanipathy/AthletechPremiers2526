package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.BetterColorSensor;

@TeleOp (group = "tuning")
public class BetterColorSensorTuner extends OpMode {

    public static float GAIN;

    public static String COLOR_SENSOR_NAME;

    private BetterColorSensor colorSensor;

    private Telemetry telemetry;


    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        colorSensor = new BetterColorSensor(hardwareMap, COLOR_SENSOR_NAME);
    }

    @Override
    public void loop() {

        colorSensor.setGain(GAIN);

        float[] rgb = colorSensor.getRGB();

        telemetry.addData("Red", rgb[0]);
        telemetry.addData("Green", rgb[1]);
        telemetry.addData("Blue", rgb[2]);
        telemetry.update();
    }
}
