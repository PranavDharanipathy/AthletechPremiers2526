package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp (group = "testing")
public class EncoderReset extends LinearOpMode {

    public static String ENCODER_NAME = "";

    private DcMotor encoder;

    @Override
    public void runOpMode() {

        encoder = hardwareMap.get(DcMotor.class, ENCODER_NAME);
        waitForStart();
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
