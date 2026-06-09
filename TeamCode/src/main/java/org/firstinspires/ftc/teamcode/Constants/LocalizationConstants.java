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

    public static FollowerConstants SOFT_FOLLOWER_CONSTANTS = new FollowerConstants()

            .mass(14.2)

            .forwardZeroPowerAcceleration(-79.89422798598751)
            .lateralZeroPowerAcceleration(-60.18572705204171)

            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(false)

            .translationalPIDFCoefficients(new PIDFCoefficients(0.2, 0, 0.03, 0.1))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.0767,0.00008,0.0045,0.04))

            .headingPIDFCoefficients(new PIDFCoefficients(1,0,0.02,0.1))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.4,0.00035,0.01,0.02))

            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.013,0.0005,0.0006,0.15,0.065))

            .translationalPIDFSwitch(3)
            .headingPIDFSwitch(0.18)
            .drivePIDFSwitch(13)

            .centripetalScaling(0.0004)

            ;

    public static FollowerConstants HARD_FOLLOWER_CONSTANTS = new FollowerConstants()

            .mass(14.2)

            .forwardZeroPowerAcceleration(-79.89422798598751)
            .lateralZeroPowerAcceleration(-60.18572705204171)

            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(false)

            .translationalPIDFCoefficients(new PIDFCoefficients(0.2, 0, 0.03, 0.1))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.0767,0.00008,0.0045,0.04))

            .headingPIDFCoefficients(new PIDFCoefficients(0.65,0,0.03,0.1))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.2,0.00035,0.02,0.02))

            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.071,0.0005,0.000615,0.15,0.065))

            .translationalPIDFSwitch(3)
            .headingPIDFSwitch(0.19)
            .drivePIDFSwitch(13)

            .centripetalScaling(0.0003)

            ;

    public static PathConstraints PATH_CONSTANTS = new PathConstraints(0.994, 50, 1, 1.4);
    public static MecanumConstants DRIVE_CONSTANTS = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName(MapSetterConstants.leftFrontMotorDeviceName)
            .leftRearMotorName(MapSetterConstants.leftBackMotorDeviceName)
            .rightFrontMotorName(MapSetterConstants.rightFrontMotorDeviceName)
            .rightRearMotorName(MapSetterConstants.rightBackMotorDeviceName)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)

            .xVelocity(70.08639598455954)
            .yVelocity(48.87915231299213);

    public static PinpointConstants LOCALIZER_CONSTANTS = new PinpointConstants()
            .forwardPodY(5.262795276)
            .strafePodX(1.34252)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName(MapSetterConstants.pinpointOdometryComputerDeviceName)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(SOFT_FOLLOWER_CONSTANTS, hardwareMap)
                .pathConstraints(PATH_CONSTANTS)
                .mecanumDrivetrain(DRIVE_CONSTANTS)
                .pinpointLocalizer(LOCALIZER_CONSTANTS)
                .build();
    }

    public static Follower createHardFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(HARD_FOLLOWER_CONSTANTS, hardwareMap)
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