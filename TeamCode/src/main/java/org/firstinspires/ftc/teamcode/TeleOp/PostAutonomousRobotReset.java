package org.firstinspires.ftc.teamcode.TeleOp;

import static android.os.SystemClock.sleep;

public class PostAutonomousRobotReset {

    public PostAutonomousRobotReset(TeleOpBaseOpMode opMode) {

        opMode.turret.setPosition(0);
        sleep(100);
        opMode.turret.setPosition(opMode.turret.startPosition);
    }
}