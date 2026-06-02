package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class CameraConstants {

    public enum PIPELINES {

        GENERAL_GOAL_PIPELINE(0),
        OBELISK_PIPELINE(1),
        TEST_PIPELINE(7);

        private int index; // default index

        PIPELINES(int index) {
            this.index = index;
        }

        public int getPipelineIndex() {
            return index;
        }

    }

    public static int CAMERA_POLL_RATE = 85;

    public static double MT1_LOCALIZATION_STEPS = 6;

    /// The maximum velocity that the robot can be traveling at to still be eligible for MT1 localization.
    /// <p>
    /// Index 0 is translational, index 1 is heading.
    public static double[] MT1_LOCALIZATION_ELIGIBILITY_MAXIMUM_ROBOT_VELOCITY = {8, 5};
}