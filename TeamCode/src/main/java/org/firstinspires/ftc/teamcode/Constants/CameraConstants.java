package org.firstinspires.ftc.teamcode.Constants;

import org.firstinspires.ftc.teamcode.Systems.CurrentAlliance;

public class CameraConstants {

    public enum PIPELINES {

        OBELISK_PIPELINE(0),
        RED_GOAL_PIPELINE(1),
        BLUE_GOAL_PIPELINE(2),
        GENERAL_GOAL_PIPELINE(3),
        TEST_PIPELINE(7);

        private int index; // default index

        PIPELINES(int index) {
            this.index = index;
        }

        public int getPipelineIndex() {
            return index;
        }

        public static PIPELINES getPipelineFromAlliance(CurrentAlliance.ALLIANCE alliance) {
            return alliance == CurrentAlliance.ALLIANCE.BLUE_ALLIANCE ? BLUE_GOAL_PIPELINE : RED_GOAL_PIPELINE;
        }
    }

    public static int CAMERA_POLL_RATE = 85;

    public static double MT1_LOCALIZATION_STEPS = 3;
    public static double ODOMETRY_RELOCALIZATION_FREQUENCY = 10; //in seconds
}