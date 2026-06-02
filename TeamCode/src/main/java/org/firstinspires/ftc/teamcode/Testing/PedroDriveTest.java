package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants.LocalizationConstants;
import org.firstinspires.ftc.teamcode.TeleOp.drive.PedroDrive;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;

@TeleOp(group = "testing")
public class PedroDriveTest extends OpMode {

    private final PedroDrive drive = new PedroDrive();

    private BetterGamepad controller1;

    @Override
    public void init() {

        controller1 = new BetterGamepad(gamepad1);

        drive.provideComponents(LocalizationConstants.createFollower(hardwareMap), controller1);
    }

    @Override
    public void loop() {

        controller1.getInformation();

        drive.update();
    }
}
