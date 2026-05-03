package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.ConfigurationConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.util.AnalogEncoder;
import org.firstinspires.ftc.teamcode.util.MathUtil;

public class HoodAngler {

    private Servo hoodServo;

    private AnalogEncoder encoder;

    public HoodAngler(HardwareMap hardwareMap, String hoodAnglerServoName) {

        encoder = new AnalogEncoder();

        hoodServo = hardwareMap.get(Servo.class, hoodAnglerServoName);
        hoodServo.setDirection(ConfigurationConstants.HOOD_ANGLER_SERVO_DIRECTION);
    }

    public Servo.Direction getServoDirection() {
        return hoodServo.getDirection();
    }

    public void setPosition(double position) {
        hoodServo.setPosition(position);
    }

    public void setSafePosition(double position) {
        hoodServo.setPosition(MathUtil.clamp(position, ShooterConstants.HOOD_ANGLER_MAX_POSITION, ShooterConstants.HOOD_ANGLER_MIN_POSITION));
    }

    public double getTargetPosition() {
        return hoodServo.getPosition();
    }

    public double getCurrentPosition() {
        return encoder.getPosition();
    }
}