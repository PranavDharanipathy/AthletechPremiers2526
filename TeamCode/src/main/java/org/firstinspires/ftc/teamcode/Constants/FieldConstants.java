package org.firstinspires.ftc.teamcode.Constants;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Systems.CurrentAlliance;
import org.firstinspires.ftc.teamcode.util.MathUtil;

public class FieldConstants {

    /**
     * x is forward-backward with forward being positive and backward being negative
     * <p>
     * y is left-right with left being positive and right being negative
     * **/
    @SuppressWarnings("all")
    public enum GoalCoordinates {

        //        CLOSE ALLIANCE       CLOSE OPPONENT            FAR
        RED(new Pose(66,66), new Pose(68,66), new Pose(61,72)),
        BLUE(new Pose(-66,66), new Pose(-68,66), new Pose(-61,72));

        public Pose closeAlliance;
        public Pose closeOpponent;
        public Pose far;

        GoalCoordinates(Pose closeAlliance, Pose closeOpponent, Pose far) {

            this.closeAlliance = closeAlliance;
            this.closeOpponent = closeOpponent;
            this.far = far;
        }

        public void setGoalCoordinates(Pose closeAlliance, Pose closeOpponent, Pose far) {

            this.closeAlliance = closeAlliance;
            this.closeOpponent = closeOpponent;
            this.far = far;
        }

        /// Added on the current pose
        public void incrementAll(double xIncPP, double yIncPP) { //PP means player perspective

            closeAlliance = new Pose(MathUtil.increaseMagnitudeNumerical(closeAlliance.getX(), yIncPP), MathUtil.increaseMagnitudeNumerical(closeAlliance.getY(), xIncPP));
            closeOpponent = new Pose(MathUtil.increaseMagnitudeNumerical(closeOpponent.getX(), yIncPP), MathUtil.increaseMagnitudeNumerical(closeOpponent.getY(), xIncPP));
            far = new Pose(MathUtil.increaseMagnitudeNumerical(far.getX(), yIncPP), MathUtil.increaseMagnitudeNumerical(far.getY(), xIncPP));
        }

        /// Added on the current pose
        public void incrementCloseAlliance(double xIncPP, double yIncPP) {
            closeAlliance = new Pose(MathUtil.increaseMagnitudeNumerical(closeAlliance.getX(), yIncPP), MathUtil.increaseMagnitudeNumerical(closeAlliance.getY(), xIncPP));
        }

        /// Added on the current pose
        public void incrementCloseOpponent(double xIncPP, double yIncPP) {
            closeOpponent = new Pose(MathUtil.increaseMagnitudeNumerical(closeOpponent.getX(), yIncPP), MathUtil.increaseMagnitudeNumerical(closeOpponent.getY(), xIncPP));
        }

        /// Added on the current pose
        public void incrementFar(double xIncPP, double yIncPP) {
            far = new Pose(MathUtil.increaseMagnitudeNumerical(far.getX(), yIncPP), MathUtil.increaseMagnitudeNumerical(far.getY(), xIncPP));
        }

        public Pose getCloseAllianceCoordinate() {
            return closeAlliance;
        }
        public Pose getCloseOpponentCoordinate() {
            return closeOpponent;
        }

        public Pose getCloseCoordinate(double x, GoalCoordinates allianceUsingGoalCoordinates) {

            boolean isOpponent = allianceUsingGoalCoordinates == BLUE ? x > RED_CLOSE_GOAL_COORDINATE_SWITCH : x < BLUE_CLOSE_GOAL_COORDINATE_SWITCH;

            return isOpponent ? closeOpponent : closeAlliance;
        }

        public Pose getCloseCoordinate(double x, CurrentAlliance.ALLIANCE alliance) {

            boolean isOpponent = alliance == CurrentAlliance.ALLIANCE.BLUE_ALLIANCE ? x > RED_CLOSE_GOAL_COORDINATE_SWITCH : x < BLUE_CLOSE_GOAL_COORDINATE_SWITCH;

            return isOpponent ? closeOpponent : closeAlliance;
        }

        public Pose getFarCoordinate() {
            return far;
        }

        // (lateral) y value after which (once y is greater) close goal coordinate switches from alliance to opponent
        public static double RED_CLOSE_GOAL_COORDINATE_SWITCH = 18;
        public static double BLUE_CLOSE_GOAL_COORDINATE_SWITCH = -18;

        public void setRedCloseGoalCoordinateSwitch(double redCloseGoalCoordinateSwitch) {
            RED_CLOSE_GOAL_COORDINATE_SWITCH = redCloseGoalCoordinateSwitch;
        }

        public void setBlueCloseGoalCoordinateSwitch(double blueCloseGoalCoordinateSwitch) {
            BLUE_CLOSE_GOAL_COORDINATE_SWITCH = blueCloseGoalCoordinateSwitch;
        }

        public static boolean onAllianceSide(double x, CurrentAlliance.ALLIANCE alliance) {

            boolean isAlliance = alliance == CurrentAlliance.ALLIANCE.BLUE_ALLIANCE ? x < BLUE_CLOSE_GOAL_COORDINATE_SWITCH : x > RED_CLOSE_GOAL_COORDINATE_SWITCH;

            return isAlliance;
        }

        public boolean onAllianceSide(double x) {

            boolean isAlliance = this == BLUE ? x < BLUE_CLOSE_GOAL_COORDINATE_SWITCH : x > RED_CLOSE_GOAL_COORDINATE_SWITCH;

            return isAlliance;
        }

        public static boolean onOpponentSide(double x, CurrentAlliance.ALLIANCE alliance) {

            boolean isOpponent = alliance == CurrentAlliance.ALLIANCE.BLUE_ALLIANCE ? x > RED_CLOSE_GOAL_COORDINATE_SWITCH : x < BLUE_CLOSE_GOAL_COORDINATE_SWITCH;

            return isOpponent;
        }

        public boolean onOpponentSide(double x) {

            boolean isOpponent = this == BLUE ? x > RED_CLOSE_GOAL_COORDINATE_SWITCH : x < BLUE_CLOSE_GOAL_COORDINATE_SWITCH;

            return isOpponent;
        }
    }

}