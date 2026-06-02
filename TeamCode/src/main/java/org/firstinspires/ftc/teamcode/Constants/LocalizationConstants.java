package org.firstinspires.ftc.teamcode.Constants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class LocalizationConstants {

    public static FollowerConstants FOLLOWER_CONSTANTS = new FollowerConstants()

            .mass(14.2)

            .forwardZeroPowerAcceleration(-24.671359021186653)
            .lateralZeroPowerAcceleration(-56.07608848166188)

            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(false)

            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.005))

            .headingPIDFCoefficients(new PIDFCoefficients(0.85,0,0.086,0.1))

            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.071,0.0005,0.000615,0.15,0.065))

            .translationalPIDFSwitch(3)
            .headingPIDFSwitch(0.19)
            .drivePIDFSwitch(13)


            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.0767,0.00009,0.0185,0.005))

            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.668,0.0007,0.085,0.025))

            .centripetalScaling(0.0004)
            ;

    public static PathConstraints PATH_CONSTANTS = new PathConstraints(0.994, 50, 1.4, 1.4);
    public static MecanumConstants DRIVE_CONSTANTS = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName(MapSetterConstants.leftFrontMotorDeviceName)
            .leftRearMotorName(MapSetterConstants.leftBackMotorDeviceName)
            .rightFrontMotorName(MapSetterConstants.rightFrontMotorDeviceName)
            .rightRearMotorName(MapSetterConstants.rightBackMotorDeviceName)
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)

            .xVelocity(82.51125978484868)
            .yVelocity(61.454884446512054);

    public static PinpointConstants LOCALIZER_CONSTANTS = new PinpointConstants()
            .forwardPodY(133.675)
            .strafePodX(34.1)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName(MapSetterConstants.pinpointOdometryComputerDeviceName)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(FOLLOWER_CONSTANTS, hardwareMap)
                .pathConstraints(PATH_CONSTANTS)
                .mecanumDrivetrain(DRIVE_CONSTANTS)
                .pinpointLocalizer(LOCALIZER_CONSTANTS)
                .build();
    }

    public static double[] ODOMETRY_STD_DEV = new double[] {
            0.0000007065919563353551,
            0.0000001802491227419467,
            0.00000013426180856367417
    };
    public static double[] CAMERA_STD_DEV = new double[] {
            0.07150979570015249,
            0.17279975433986944,
            0.00016164960755747092
    };
    public static double[] PROCESS_NOISE_STD_DEV = new double[] {0.08, 0.08, 0.01, 0.6, 0.6, 0.3};

}