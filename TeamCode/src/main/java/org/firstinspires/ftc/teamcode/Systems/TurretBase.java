package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.util.DynamicTrapezoidalSum;
import org.firstinspires.ftc.teamcode.util.LowPassFilter;
import org.firstinspires.ftc.teamcode.util.MathUtil;

import static org.firstinspires.ftc.teamcode.Constants.ConfigurationConstants.TURRET_PD_POSITIONS;
import static org.firstinspires.ftc.teamcode.Constants.ConfigurationConstants.TURRET_KPS;
import static org.firstinspires.ftc.teamcode.Constants.ConfigurationConstants.TURRET_KDS;

import static org.firstinspires.ftc.teamcode.Constants.ConfigurationConstants.TURRET_FEEDFORWARD_POSITIONS;
import static org.firstinspires.ftc.teamcode.Constants.ConfigurationConstants.TURRET_KFS;

import java.util.Collections;

public class TurretBase {

    private final TurretActuator turretActuator;

    public TurretActuator getActuator() {
        return turretActuator;
    }

    public double kp, kiFar, kiClose, kd, kISmash, kDFilter, kVelocityFilter;
    public double ki, kHold;
    public double holdDecay;

    private double maxI = 1;
    private double minI = -1;

    public double[] dActivation = {0, (double) Integer.MAX_VALUE};

    private double[] iSwitch;

    private DynamicTrapezoidalSum errorSum = new DynamicTrapezoidalSum();

    public double p, i, d = 0, f;
    private double filteredDerivative = 0;

    public double filteredPositionalTargetVelocity = 0;

    private final VoltageSensor batteryVoltageSensor;

    public TurretBase(HardwareMap hardwareMap) {

        this (hardwareMap, null); //using encoder's current position
    }

    /// @param turretStartPosition for re-zeroing the turret compensating for the home position not always being at 0
    public TurretBase(HardwareMap hardwareMap, Double turretStartPosition) {

        turretActuator = new TurretActuator(hardwareMap);
        turretActuator.setSMode(TurretActuator.SMode.POSITION_ERROR);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // first targetPosition is the start position
        double tsp = turretStartPosition != null ? turretStartPosition : turretActuator.getStartPosition();
        lastCurrentPosition = currentPosition = lastTargetPosition = targetPosition = startPosition = tsp;
    }

    private boolean reversed = false;

    /// Call after setting PIDFS coefficients
    public void reverse() {

        DcMotorSimple.Direction direction = turretActuator.getDirection() == DcMotorSimple.Direction.FORWARD ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;

        turretActuator.setDirection(direction);

        TURRET_PD_POSITIONS.replaceAll(i -> -i);
        TURRET_FEEDFORWARD_POSITIONS.replaceAll(i -> -i);

        Collections.reverse(TURRET_PD_POSITIONS);
        Collections.reverse(TURRET_FEEDFORWARD_POSITIONS);

        Collections.reverse(TURRET_KPS);
        Collections.reverse(TURRET_KDS);
        Collections.reverse(TURRET_KFS);

        reversed = true;
    }

    public void setVelocityCoefficients(double[] velocityCoefficients) {

        turretActuator.setPIDVSCoefficients(
                velocityCoefficients[0],
                velocityCoefficients[1],
                velocityCoefficients[2],
                velocityCoefficients[3],
                velocityCoefficients[4],
                velocityCoefficients[5],
                velocityCoefficients[6],
                velocityCoefficients[7],
                velocityCoefficients[8],
                velocityCoefficients[9],
                velocityCoefficients[10],
                velocityCoefficients[11],
                velocityCoefficients[12],
                velocityCoefficients[13],
                velocityCoefficients[14],
                velocityCoefficients[15],
                velocityCoefficients[16]
        );
    }

    public TurretBasePIDFCoefficients coefficients;

    public void setPositionalCoefficients(TurretBasePIDFCoefficients coefficients) {

        this.coefficients = coefficients;

        //setting variables that do not change

        iSwitch = coefficients.iSwitch;

        holdDecay = coefficients.holdDecay;

        kVelocityFilter = coefficients.kVelocityFilter;

        kDFilter = coefficients.kDFilter;

        dActivation = coefficients.dActivation;

        minI = coefficients.minI;
        maxI = coefficients.maxI;
    }

    /// @param tuning true means that turret's in tuning mode while false means that turret is in normal mode.
    /// If the object isn't initialized, nothing will happen and the method will deal with the error.
    public void setTuning(boolean tuning) {
        coefficients.setTuning(tuning);
    }

    public enum PD_INTERPOLATION_MODE {

        NONE, P, D, BOTH;

        /**
         * <p>"00" - Indicates NONE
         * <p>"10" - Indicates P
         * <p>"01" - Indicates D
         * <p>"11" - Indicates BOTH
         * @throws IllegalArgumentException If an invalid mode is inputted
         */
        public static PD_INTERPOLATION_MODE fromString(String mode) {

            switch (mode) {

                case "00":
                    return NONE;

                case "10":
                    return P;

                case "01":
                    return D;

                case "11":
                    return BOTH;

                default:
                    throw new IllegalArgumentException(mode + " is an invalid mode string!");
            }
        }
    }

    public PD_INTERPOLATION_MODE pdInterpolationMode = PD_INTERPOLATION_MODE.BOTH;

    /// To be able to set from FTC Dashboard
    public void setPdInterpolationMode(PD_INTERPOLATION_MODE mode) {
        pdInterpolationMode = mode;
    }

    /// Setting variables that do in fact change
    private void chooseCoefficientsInternal(TurretBasePIDFCoefficients.TurretSide side) {

        if (pdInterpolationMode.equals(PD_INTERPOLATION_MODE.BOTH)) {

            double[] kpAndKd = coefficients.kpAndKd(targetPosition, currentPosition, startPosition);
            kp = kpAndKd[0];
            kd = kpAndKd[1];
        }
        else { //if pdInterpolationMode equals PD_INTERPOLATION_MODE.P, PD_INTERPOLATION_MODE.D, or PD_INTERPOLATION_MODE.NONE

            kp = coefficients.kp(targetPosition, currentPosition, startPosition);
            kd = coefficients.kd(targetPosition, currentPosition, startPosition);
        }

        kiFar = coefficients.kiFar(side);
        kiClose = coefficients.kiClose(side);
        kHold = coefficients.kHold(targetPosition, startPosition, batteryVoltageSensor.getVoltage());

        kISmash = coefficients.kISmash(side);
    }

    public double startPosition;
    private double lastTargetPosition;
    private double targetPosition;

    private double currentPosition;
    private double lastCurrentPosition;

    public void setPosition(double position) {

        if (targetPosition != position) {

            lastTargetPosition = targetPosition;
            targetPosition = position;

            initialError = null; //null means that it's to be determined
        }
    }

    private double travelVelocity;

    public void setVelocity(double velocity) {
        if (travelVelocity != velocity) travelVelocity = velocity;
    }

    public double getLastTargetPosition() {
        return lastTargetPosition;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public double getLastCurrentPosition() {
        return lastCurrentPosition;
    }

    public double getCurrentPosition() {
        return turretActuator.getCurrentPosition();
    }

    private double prevError, error;
    private Double initialError = null;
    private double prevTime, currTime, dt;

    private final ElapsedTime timer = new ElapsedTime();

    public void update() {

        lastCurrentPosition = currentPosition;
        currentPosition = getCurrentPosition();

        currTime = timer.milliseconds();
        dt = currTime - prevTime;

        error = targetPosition - currentPosition;

        if (initialError == null) initialError = error;

        chooseCoefficientsInternal(TurretBasePIDFCoefficients.TurretSide.getSide(targetPosition, startPosition, reversed));

        boolean fineTune = Math.abs(error) < iSwitch[0];

        //proportional
        p = kp * error;

        //integral
        if (Math.abs(error) > iSwitch[1]) ki = 0;
        else if (fineTune) ki = kiClose;
        else ki = kiFar;

        if (dt != 0) errorSum.updateSum(dt, error);
        if (Math.signum(error) != Math.signum(prevError)) {
            errorSum.setSum(errorSum.getSum() * kISmash);
        }
        errorSum.setRawSum(MathUtil.clamp(errorSum.getSum(), (minI / ki) /*integrated error min*/, (maxI / ki) /*integrated error max*/));
        i = ki * errorSum.getSum();
        //i = MathUtil.clamp(ki * errorSum.getSum(), minI, maxI);

        //derivative
        double rawDerivative = (error - prevError) / dt;
        filteredDerivative = LowPassFilter.getFilteredValue(filteredDerivative, rawDerivative, kDFilter);
        d = dt > 0 && MathUtil.valueWithinRange(Math.abs(error), dActivation[0], dActivation[1]) ? kd * filteredDerivative : 0;

        //feedforward
        f = kHold * Math.signum(error) * (1.0 - Math.exp(-Math.abs(error) / (holdDecay * ShooterConstants.TURRET_TICKS_PER_DEGREE)));

        double rawPositionalTargetVelocity = p + i + d;
        filteredPositionalTargetVelocity = LowPassFilter.getFilteredValue(filteredPositionalTargetVelocity, rawPositionalTargetVelocity, kVelocityFilter);

        turretActuator.providePositionError(error);
        turretActuator.setVelocity(filteredPositionalTargetVelocity + travelVelocity);
        turretActuator.setAdditionalPower(f);
        turretActuator.update();

        prevTime = currTime;
        prevError = error;
    }

    public double getErrorMagnitude() {
        return Math.abs(error);
    }

    public double getError() {
        return error;
    }

    /// @return What the error was initially when the PID started working towards the new target position
    public double getInitialError() {
        return initialError;
    }

    public double getActuatorTargetVelocity() {
        return filteredPositionalTargetVelocity;
    }

    public double getTravelVelocity() {
        return travelVelocity;
    }

    public double getVelocity() {
        return turretActuator.getCurrentVelocity();
    }

    public double getPower() {
        return turretActuator.getPower();
    }

    public double getLoopDt() {
        return MathUtil.millisecondsToSeconds(dt);
    }

    public double[] getServoPowers() {
        return new double[] {turretActuator.getLeftTurretBase().getPower(), turretActuator.getRightTurretBase().getPower()};
    }

}