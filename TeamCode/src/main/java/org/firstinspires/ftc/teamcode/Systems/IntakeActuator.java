package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.ConfigurationConstants;
import org.firstinspires.ftc.teamcode.Constants.MapSetterConstants;

public class IntakeActuator {

    private DcMotorEx intake, transfer;

    public IntakeActuator(HardwareMap hardwareMap) {

        intake = hardwareMap.get(DcMotorEx.class, MapSetterConstants.intakeMotorDeviceName);
        transfer = hardwareMap.get(DcMotorEx.class, MapSetterConstants.transferMotorDeviceName);

        intake.setDirection(ConfigurationConstants.INTAKE_MOTOR_DIRECTION);
        transfer.setDirection(ConfigurationConstants.TRANSFER_MOTOR_DIRECTION);

        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void setVelocityPDFCoefficients(double kp, double kd, double kff) {

        this.kp = kp;
        this.kd = kd;
        this.kff = kff;
    }

    private double intakePower;

    private double transferTargetVelocity, targetCurrentVelocity;
    private double transferPower;

    public enum TransferControl {
        PID, POWER
    }

    private TransferControl transferControl = TransferControl.PID;

    public void setTransferVelocity(double velocity) {

        transferTargetVelocity = velocity;
        transferControl = TransferControl.PID;
    }

    public void setTransferPower(double power) {
        
        transferPower = power;
        transferControl = TransferControl.POWER;
    }

    public void setIntakePower(double power) {
        intakePower = power;
    }

    public double getTransferVelocity() {
        return targetCurrentVelocity;
    }

    private double kp, kd, kff;
    public double p, d, ff;
    private double prevTime = 0, currTime = 0;
    private double prevError = 0, error = 0;

    private double prevPosition, currPosition = 0;

    private double startTime = 0;
    private double getSeconds() {
        return System.nanoTime() * 1e-9 - startTime;
    }

    private boolean firstTick = true;

    public void update() {

        if (firstTick) {

            startTime = getSeconds();
            firstTick = false;
            return;
        }

        prevTime = currTime;
        currTime = getSeconds();

        double dt = currTime - prevTime;

        prevPosition = currPosition;
        currPosition = transfer.getCurrentPosition();

        targetCurrentVelocity = (currPosition - prevPosition) / dt;

        prevError = error;
        error = transferTargetVelocity - targetCurrentVelocity;

        double transferPower;
        if (transferControl == TransferControl.PID) {

            //proportional
            p = kp * error;

            //derivative
            d = dt > 0 ? kd * (error - prevError) / dt : 0;

            //full feedforward
            ff = kff * transferTargetVelocity;

            transferPower = p + d + ff;
        }
        else {
            transferPower = this.transferPower;
        }

        intake.setPower(intakePower);
        transfer.setPower(transferPower);
    }

    public double getIntakePower() {
        return intake.getPower();
    }

    public double getTransferPower() {
        return transfer.getPower();
    }

}
