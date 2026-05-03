package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Constants.ConfigurationConstants;
import org.firstinspires.ftc.teamcode.util.DoubleM;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.MathUtil;

public final class Flywheel {

    private final Encoder encoder;

    public Encoder getEncoder() {
        return encoder;
    }

    private final DcMotorEx leftFlywheel;
    private final DcMotorEx rightFlywheel; //follows leftFlywheel

    private VoltageSensor batteryVoltageSensor;

    public Flywheel(DcMotorEx leftFlywheel, DcMotorEx rightFlywheel) {

        this.leftFlywheel = leftFlywheel;
        this.rightFlywheel = rightFlywheel;

        this.leftFlywheel.setDirection(ConfigurationConstants.FLYWHEEL_MOTOR_DIRECTIONS[0]);
        this.rightFlywheel.setDirection(ConfigurationConstants.FLYWHEEL_MOTOR_DIRECTIONS[1]);

        encoder = new Encoder(this.rightFlywheel);
        encoder.setDirection(Encoder.Direction.FORWARD);

        encoder.setupVelocityKalmanFilter(ConfigurationConstants.FLYWHEEL_KALMAN_FILTER_PARAMETERS);

        this.leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void initVoltageSensor(HardwareMap hardwareMap) {
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public double kp;
    public double ki;
    public double kISmash;
    public double kd;
    public double kv;
    public double ks;

    private double kPIDFUnitsPerVolt;

    private double VbackEMF;

    private double MOTOR_RPM;

    private double FN;

    private double SHAFT_RADIUS;

    private double targetVelocity;
    private double currentVelocity, velocityEstimate;

    private double lastCurrentVelocity = 0;

    /// @param MASS_IN_GRAMS Is the amount of mass in grams that is connected to the motor.
    /// @param SHAFT_DIAMETER Is the diameter of shaft in millimeters connecting to motor.
    /// @param MOTOR_CORE_VOLTAGE Check the website you go the motor from, it may tell you what volt motor core the motor has.
    /// @param MOTOR_RPM Is the RPM of the motor.
    public void setInternalParameters(double MASS_IN_GRAMS, double SHAFT_DIAMETER, double MOTOR_CORE_VOLTAGE, double MOTOR_RPM) {

        this.MOTOR_RPM = MOTOR_RPM;

        VbackEMF = MOTOR_CORE_VOLTAGE;

        this.SHAFT_RADIUS = SHAFT_DIAMETER / 2;
        FN = /*gravity*/ 9.80665 * (/*converted mass in g to kg*/ MASS_IN_GRAMS / 1000);

    }

    public double p = 0, i = 0, d = 0;
    private DoubleM errorSum = new DoubleM(0);
    public double v = 0;
    public double s = 0;

    public double minP, maxP;
    public double minI, maxI;
    public double minD, maxD;

    public double[] getPIDVS() {

        return new double[] {p, i, d, v, s};
    }

    private FlywheelPIDVSCoefficients coefficients;

    public FlywheelPIDVSCoefficients getCoefficients() {
        return coefficients;
    }

    public void setVelocityPIDVSCoefficients(FlywheelPIDVSCoefficients coefficients) {

        this.coefficients = coefficients;

        kd = coefficients.kd;

        ks = coefficients.ks;
        kPIDFUnitsPerVolt = coefficients.kPIDFUnitsPerVolt;

        kISmash = coefficients.kISmash;

        minP = coefficients.minP;
        maxP = coefficients.maxP;
        minI = coefficients.minI;
        maxI = coefficients.maxI;
        minD = coefficients.minD;
        maxD = coefficients.maxD;
    }

    /// Setting variables that do in fact change
    private void chooseCoefficientsInternal() {

        kp = coefficients.kp(targetVelocity, currentVelocity);

        ki = coefficients.ki(targetVelocity, currentVelocity, errorSum);

        kv = coefficients.kv(batteryVoltageSensor);
    }

    private double currentTime = 0;

    private double prevTime = 0, prevError = 0;

    /// @param velocity in ticks per second
    public void setVelocity(double velocity, boolean allowIntegralReset) {

        if (allowIntegralReset && targetVelocity != velocity) {

            resetIntegral(); //resetting integral when target velocity changes to prevent integral windup
        }

        targetVelocity = velocity;
    }

    private double position = 0;
    private boolean firstTick = true;
    private double startTime;

    private double dt;

    private double getSeconds() {
        return System.nanoTime() * 1e-9;
    }

    private double power = 0;

    public void update() {

        //setting start time
        if (firstTick) {

            startTime = getSeconds();
            firstTick = false;
        }

        chooseCoefficientsInternal();

        double lastPosition = position;
        position = encoder.getCurrentPosition();
        prevTime = currentTime;
        currentTime = getSeconds() - startTime;
        dt = currentTime - prevTime;

        lastCurrentVelocity = currentVelocity;
        velocityEstimate = (position - lastPosition) / dt;
        //currentVelocity = velocityEstimate;
        encoder.runVelocityCalculation(velocityEstimate);
        currentVelocity = encoder.getFilteredVelocity();

        double error = targetVelocity - currentVelocity;

        //proportional
        p = kp * error;
        p = MathUtil.clamp(p, minP, maxP);

        if (!Double.isNaN(error * dt) && targetVelocity != 0) errorSum.add(error * dt);
        else errorSum.set(0); //integral is reset if it's NaN or if targetVelocity is equal to 0

        // i smashing
        if (Math.signum(error) != Math.signum(prevError)) {
            errorSum.multiply(kISmash);
        }

        // i is prevented from getting too high or too low
        i = MathUtil.clamp(ki * errorSum.get(), minI, maxI);

        //derivative
        d = dt > 0 ? kd * (error - prevError) / dt : 0;
        if (!MathUtil.valueWithinRangeIncludingPoles(d, -1 /*minimum power*/, 1 /*maximum power*/)) d = 0;
        d = MathUtil.clamp(d, minD, maxD);

        //velocity feedforward
        v = kv * targetVelocity;

        //static friction
        double freeSpeed = (MOTOR_RPM * Math.PI) / 30; // in rad/s
        double ke = VbackEMF / freeSpeed; // using ke instead of kt - #1 ks will compensate, #2 ke can more easily be calculated accurately
        double T = ks * FN * SHAFT_RADIUS;
        s = targetVelocity != 0 ? (T / ke) * kPIDFUnitsPerVolt * (error >= 0 ? 1 : 0 /*engages or turns off*/) : 0;

        power = p + i + d + v + s;

        if (targetVelocity == 0) power = 0;

        if (isPIDEnabled) {
            leftFlywheel.setPower(power);
            rightFlywheel.setPower(power);
        }

        prevError = error;
    }

    public enum RunningMotor {

        DISABLE(false), ENABLE(true);

        private boolean value;

        RunningMotor(boolean enableOrDisable) {
            value = enableOrDisable;
        }

        public boolean getValue() {
            return value;
        }
    }

    public void setPower(double power) {

        if (isPIDEnabled) throw new IllegalArgumentException("Must disable PID mode!");

        leftFlywheel.setPower(power);
        rightFlywheel.setPower(power);
    }

    // default mode is enabled
    private boolean isPIDEnabled = true;

    /// @return if motor is enabled
    public boolean getPIDEnabled() {
        return isPIDEnabled;
    }

    public void runMotor(RunningMotor isPIDEnabled) {
        this.isPIDEnabled = isPIDEnabled.getValue();
    }

    public double getVelocityEstimate() {
        return velocityEstimate;
    }

    public double getCurrentVelocity() {
        return currentVelocity;
    }

    public double getLastCurrentVelocity() {
        return lastCurrentVelocity;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    /// @param velocityMarginOfError Acceptable variation in velocity.
    /// @param stabilityMarginOfError Acceptable variation in stability.
    public boolean isAtVelocityAndStable(double velocityMarginOfError, double stabilityMarginOfError) {

        double currentSpeed = Math.abs(currentVelocity);
        double lastSpeed = Math.abs(lastCurrentVelocity);

        boolean isAtVelocity = Math.abs(targetVelocity - currentSpeed) <= velocityMarginOfError;

        boolean isStable = Math.abs(currentSpeed - lastSpeed) <= stabilityMarginOfError;

        return isAtVelocity && isStable;
    }

    public void reset() {

        currentTime = 0;

        prevError = 0;
        prevTime = 0;

        targetVelocity = 0;
        currentVelocity = 0;

        lastCurrentVelocity = 0;

        setVelocity(0, false); //allowIntegralReset is false to speed up computation because of how '||' works - probably negligible
        resetIntegral(); //integral reset
    }

    private void resetIntegral() {
        errorSum.set(0);
        i = 0;
    }

    public double getPower() {
        return power;
    }

    public double[] getMotorPowers() {
        return new double[] {leftFlywheel.getPower(), rightFlywheel.getPower()};
    }

    /// @return the dt used in the loop that ran most recently
    public double getLoopDt() {
        return dt;
    }

}