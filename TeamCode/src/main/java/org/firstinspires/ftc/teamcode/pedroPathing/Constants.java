package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.06)
            .forwardZeroPowerAcceleration(-43.78916915773277)
            .lateralZeroPowerAcceleration(-72.4548420459328)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0, 0))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.15,0,0.01,0.015))
            .headingPIDFCoefficients(new PIDFCoefficients(2.5, 0, 0, 0))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.5, 0, 0.08, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.3,0.0,0.000015,0.6,0.01))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.02,0,0.000005,0.6,0.01))
            .centripetalScaling(0.0004);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("bottomRight")
            .leftRearMotorName("bottomLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(73.46923795369241)
            .yVelocity(53.819836830111988);

    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("bottomRight")
            .strafeEncoder_HardwareMapName("frontLeft")
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )
            )
            .strafePodX(5.429)//5.875
            .forwardPodY(-1.539)
            .forwardEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.REVERSE)
            .forwardTicksToInches(0.0019608748295770863)
            .strafeTicksToInches(0.0019431981339870267);

    public static OTOSConstants localizerConstantsAlt = new OTOSConstants()
            .hardwareMapName("otos")
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.RADIANS)
            .offset(new SparkFunOTOS.Pose2D(-2.625,0.625, Math.toRadians(270)))
            .linearScalar(0.95699424974) //1. 0.9619731663857453 2. 0.94401572778275797 3. 0.9649938550724638
            .angularScalar(0.997213); //1. 0.99668 2. 0.997746

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .OTOSLocalizer(localizerConstantsAlt)
//                .twoWheelLocalizer(localizerConstantsAlt)
                .build();
    }
}
