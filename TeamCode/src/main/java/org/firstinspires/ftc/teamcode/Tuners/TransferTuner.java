package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Systems.IntakeActuator;

@Config
@TeleOp(group = "tuning")
public class TransferTuner extends OpMode {

    public static double KP;
    public static double KD;
    public static double KFF;

    public static double VELOCITY;

    private IntakeActuator intake;

    private Telemetry telemetry;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        intake = new IntakeActuator(hardwareMap);

    }

    @Override
    public void loop() {

        intake.setVelocityPDFCoefficients(KP, KD, KFF);

        intake.setTransferVelocity(VELOCITY);

        intake.update();

        telemetry.addData("velocity", intake.getTransferVelocity());
        telemetry.addData("power", intake.getTransferPower());
        telemetry.update();
    }
}
