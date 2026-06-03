package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.teamcode.Constants.ConfigurationConstants;
import org.firstinspires.ftc.teamcode.Constants.MapSetterConstants;
import org.firstinspires.ftc.teamcode.util.DynamicTrapezoidalSum;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.LowPassFilter;
import org.firstinspires.ftc.teamcode.util.MathUtil;

public class TurretActuator {

    private final CRServoImplEx leftTurretBase, rightTurretBase;

    public CRServoImplEx getLeftTurretBase() {
        return leftTurretBase;
    }

    public CRServoImplEx getRightTurretBase() {
        return rightTurretBase;
    }

    private final Encoder encoder;

    public Encoder getEncoder() {
        return encoder;
    }

    private final VoltageSensor batterVoltageSensor;

    private final boolean useVelocityKalmanFilter;

    public TurretActuator(HardwareMap hardwareMap, boolean useVelocityKalmanFilter) {

        leftTurretBase = hardwareMap.get(CRServoImplEx.class, MapSetterConstants.turretBaseLeftServoDeviceName);
        rightTurretBase = hardwareMap.get(CRServoImplEx.class, MapSetterConstants.turretBaseRightServoDeviceName);

        leftTurretBase.setPwmEnable();
        rightTurretBase.setPwmEnable();

        leftTurretBase.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rightTurretBase.setPwmRange(new PwmControl.PwmRange(500, 2500));

        leftTurretBase.setDirection(ConfigurationConstants.TURRET_BASE_DIRECTIONS[0]);
        rightTurretBase.setDirection(ConfigurationConstants.TURRET_BASE_DIRECTIONS[1]);

        encoder = new Encoder(hardwareMap.get(DcMotorEx.class, MapSetterConstants.turretExternalEncoderMotorPairName));
        encoder.setDirection(Encoder.Direction.REVERSE);

        batterVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        filteredVoltage = batterVoltageSensor.getVoltage();

        this.useVelocityKalmanFilter = useVelocityKalmanFilter;
        if (this.useVelocityKalmanFilter) encoder.setupVelocityKalmanFilter(ConfigurationConstants.TURRET_KALMAN_FILTER_PARAMETERS);

        prevPosition = currentPosition = startPosition = encoder.getCurrentPosition();
    }

    public TurretActuator(HardwareMap hardwareMap) {
        this (hardwareMap, true);
    }

    public void setPIDVSCoefficients(double kpClose, double kpFar, double sharpness, double ki, double kISmash, double minI, double maxI, double kd, double nominalDt, double unscaledKvLeft, double unscaledKvRight, double tuningVoltage, double voltageFilterAlpha, double kStatic, double kCoulomb, double kB, double kStribeckVelocity) {

        this.kpClose = kpClose;
        this.kpFar = kpFar;
        this.sharpness = sharpness;
        this.ki = ki;
        this.kISmash = kISmash;
        this.minI = minI;
        this.maxI = maxI;
        this.kd = kd;
        this.nominalDt = nominalDt;
        this.unscaledKvLeft = unscaledKvLeft;
        this.unscaledKvRight = unscaledKvRight;
        this.tuningVoltage = tuningVoltage;
        this.voltageFilterAlpha = voltageFilterAlpha;
        this.kStatic = kStatic;
        this.kCoulomb = kCoulomb;
        this.kB = kB;
        this.kStribeckVelocity = kStribeckVelocity;
    }

    public void setVelocity(double velocity) {

        if (targetVelocity == velocity) return;

        lastTargetVelocity = targetVelocity;
        targetVelocity = velocity;

        if ((targetVelocity - lastTargetVelocity) > ConfigurationConstants.TURRET_MIN_TARGET_VELOCITY_CHANGE_REQUIRING_INTEGRAL_RESET) {
            errorSum.setSum(0);
        }
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        leftTurretBase.setDirection(direction);
        rightTurretBase.setDirection(direction);
    }

    public DcMotorSimple.Direction getDirection() {
        return leftTurretBase.getDirection();
    }

    public double getPower() {
        return leftTurretBase.getPower();
    }

    private double prevError = 0, error;

    public double getError() {
        return error;
    }

    private final ElapsedTime timer = new ElapsedTime();
    private double prevTime, currTime = 0, dt;

    private double startPosition;

    private double prevPosition, currentPosition;

    private double lastTargetVelocity = 0, targetVelocity = 0;
    private double velocityEstimate, currentVelocity;

    public double getLastTargetVelocity() {
        return lastTargetVelocity;
    }
    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getVelocityEstimate() {
        return velocityEstimate;
    }

    public double getCurrentVelocity() {
        return currentVelocity;
    }

    public double getCurrentPosition() {
        return encoder.getCurrentPosition();
    }

    public double p, i, d = 0, v, s;
    private final DynamicTrapezoidalSum errorSum = new DynamicTrapezoidalSum();

    public double kpClose, kpFar;
    private double sharpness;
    private double kp;
    public double getKp() {
        return kp;
    }

    private double ki, kISmash, minI, maxI;

    private double rawDerivative, kd, nominalDt;

    private double unscaledKvLeft, unscaledKvRight, tuningVoltage, voltageFilterAlpha;
    private double filteredVoltage;

    private double kStatic, kCoulomb, kB, kStribeckVelocity;

    public void update() {

        prevTime = currTime;
        currTime = timer.seconds();
        dt = currTime - prevTime;

        prevPosition = currentPosition;
        currentPosition = getCurrentPosition();

        velocityEstimate = (currentPosition - prevPosition) / dt;

        if (useVelocityKalmanFilter) {
            encoder.runVelocityCalculation(velocityEstimate);
            currentVelocity = encoder.getFilteredVelocity();
        }
        else {
            currentVelocity = velocityEstimate;
        }

        error = targetVelocity - currentVelocity;

        //proportional
        kp = kpClose + kpFar * FastMath.tanh(sharpness * Math.abs(error));
        kp = MathUtil.clamp(kp, -1, 1);

        p = kp * error;

        //integral
        if (dt != 0) errorSum.updateSum(dt, error);
        if (Math.signum(error) != Math.signum(prevError) && error != 0) {
            errorSum.setSum(errorSum.getSum() * kISmash);
        }
        errorSum.setRawSum(MathUtil.clamp(errorSum.getSum(), (minI / ki) /*integrated error min*/, (maxI / ki) /*integrated error max*/));
        i = ki * errorSum.getSum();

        //derivative
        rawDerivative = (error - prevError) / dt;
        d = LowPassFilter.getFilteredValue(d, rawDerivative, nominalDt / (nominalDt + dt), kd / (nominalDt + dt));

        //velocity feedforward
        filteredVoltage = LowPassFilter.getFilteredValue(filteredVoltage, batterVoltageSensor.getVoltage(), voltageFilterAlpha);

        double reZeroedTargetPosition = currentPosition - startPosition;

        double unscaledKv = reZeroedTargetPosition >= 0 ? unscaledKvRight : unscaledKvLeft;
        double scaledKv = (tuningVoltage / filteredVoltage) * unscaledKv;
        v = scaledKv * targetVelocity;

        //friction feedforward
        double sControl = sMode == SMode.TARGET_VELOCITY ? targetVelocity : positionError;
        double gaussianDecayCalculationExponent = (targetVelocity != 0 || kStribeckVelocity != 0) ? -((targetVelocity / kStribeckVelocity) * (targetVelocity / kStribeckVelocity)) : 0;
        s = kCoulomb * Math.signum(sControl) + ((tuningVoltage / filteredVoltage) * kB) * targetVelocity + (kStatic - kCoulomb) * Math.signum(sControl) * (FastMath.pow(Math.E, gaussianDecayCalculationExponent));

        if (PIDEnabled) setPower(p + i + d + v + s + additionalPower);

        prevError = error;
    }

    public enum SMode {
        TARGET_VELOCITY, POSITION_ERROR
    }

    private SMode sMode = SMode.TARGET_VELOCITY;

    public void setSMode(SMode sMode) {
        this.sMode = sMode;
    }

    private double positionError = 0;

    public void providePositionError(double positionError) {
        this.positionError = positionError;
    }

    private double additionalPower = 0;

    public void setAdditionalPower(double power) {
        additionalPower = power;
    }

    public double getRawDerivative() {
        return rawDerivative;
    }

    private boolean PIDEnabled = true;

    public void setPIDUsage(boolean enable) {
        PIDEnabled = enable;
    }

    public void setPower(double power) {

        leftTurretBase.setPower(power);
        rightTurretBase.setPower(power);
    }

    public double getStartPosition() {
        return startPosition;
    }

    public double getLoopDt() {
        return dt;
    }

    public double[] getServoPowers() {
        return new double[] {
                leftTurretBase.getPower(), rightTurretBase.getPower()
        };
    }

}
