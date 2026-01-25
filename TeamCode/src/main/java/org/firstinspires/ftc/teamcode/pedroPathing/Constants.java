
package org.firstinspires.ftc.teamcode.pedroPathing;

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

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.977)
            .forwardZeroPowerAcceleration(-32.43139400803568)
            .lateralZeroPowerAcceleration(-57.84886133459925)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.2,0,0.03,0.025))
            .headingPIDFCoefficients(new PIDFCoefficients(0.6,0,0.0002,0.03))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.06,0,0.00001,0.6,0.01))
            ;
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1.0)
            .rightFrontMotorName("fRight")
            .rightRearMotorName("bRight")
            .leftRearMotorName("bLeft")
            .leftFrontMotorName("fLeft")

            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)

            .xVelocity(72.76072104146162)
            .yVelocity(40.98732079320793)

            ;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-6.235)
            .strafePodX(-7.3)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);



    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
