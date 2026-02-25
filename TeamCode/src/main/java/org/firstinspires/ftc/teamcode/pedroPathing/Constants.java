
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
            .mass(11.3398)
            .forwardZeroPowerAcceleration(37.40912778656123)
            .lateralZeroPowerAcceleration(67.07079274581157)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.06, 0, 0.001, 0.0035))
            .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0.01, 0.02))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.4,0, 0.00001,0.6,0.01))
            .centripetalScaling(0.0006)
            ;
    public static PathConstraints pathConstraints = new PathConstraints(0.99,
            100,
            1,
            1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("fRight")
            .rightRearMotorName("bRight")
            .leftRearMotorName("bLeft")
            .leftFrontMotorName("fLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(73.07059941329356)
            .yVelocity(54.48474841981422);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-48/2.54)
            .strafePodX(59.5/2.54)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}