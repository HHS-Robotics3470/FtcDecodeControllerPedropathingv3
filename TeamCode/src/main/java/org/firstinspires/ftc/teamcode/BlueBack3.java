package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subSystems.Outtake;
import org.firstinspires.ftc.teamcode.subSystems.Spindexer;
import org.firstinspires.ftc.teamcode.subSystems.Vision;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "Blue Back 3", group = "Autonomous")
public class BlueBack3 extends OpMode {

    private Vision vision;
    private Outtake outtake;
    private Spindexer spindexer;
    private Follower follower;
    private Paths paths;

    private int pathState = 0;
    private int shootState = -1;
    private int patternId = -1;
    private long timer = 0;

    private static final long FLYWHEEL_SPINUP_TIME = 5000;
    private static final long FEED_TO_SHOOT_DELAY = 600;
    private static final long ARM_UP_TIME = 150;
    private static final long POST_DOWN_DELAY = 0;

    @Override
    public void init() {
        vision = new Vision();
        vision.init(hardwareMap);
        vision.setValidIds(21, 22, 23);

        outtake = new Outtake();
        outtake.init(hardwareMap);

        spindexer = new Spindexer();
        spindexer.init(hardwareMap);

//      follower = Constants.createFollower(hardwareMap);
//      follower.setStartingPose(new Pose(51, 123, Math.toRadians(90)));

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(new Pose(51, 123, Math.toRadians(90)));

        paths = new Paths(follower);

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        vision.updateVision();
//      follower.update();
        follower.updatePose();
        follower.update();
        PoseStorage.currentPose = follower.getPose();

        if (patternId == -1 && vision.getTargetId() != -1) {
            patternId = vision.getTargetId();
        }

        autonomousPathUpdate();

        if (pathState == 4 && patternId != -1) {
            switch (patternId) {
                case 21:
                    runShootingSequence(new int[]{2, 1, 3});
                    break;
                case 22:
                    runShootingSequence(new int[]{1, 2, 3});
                    break;
                case 23:
                    runShootingSequence(new int[]{1, 3, 2});
                    break;
            }
        }

        telemetry.addData("Pattern ID", patternId);
        telemetry.addData("PathState", pathState);
        telemetry.addData("ShootState", shootState);
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    // ================= PATH FSM =================
    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.toSee);
                pathState = 1;
                break;

            case 1:
                if (!follower.isBusy()) pathState = 2;
                break;

            case 2:
                if (patternId != -1) {
                    follower.followPath(paths.toShoot);
                    pathState = 3;
                }
                break;

            case 3:
                // wait until the rotate-and-slide path finishes
                if (!follower.isBusy()) pathState = 4;
                break;

            case 4:
                break; // shooting FSM

            case 5:
                follower.followPath(paths.toPark);
                pathState = 6;
                break;

            case 6:
                if (!follower.isBusy()) pathState = 7;
                break;

            case 7:
                break;
        }
    }

    // ================= SHOOTING FSM =================
    private void runShootingSequence(int[] ballOrder) {

        if (shootState == -1) {
            timer = System.currentTimeMillis();
            shootState = 0;
            return;
        }

        if (shootState == 0) {
            if (System.currentTimeMillis() - timer >= FLYWHEEL_SPINUP_TIME) {
                shootState = 1;
                timer = System.currentTimeMillis();
            }
            return;
        }

        int ringIndex = (shootState - 1) / 4;
        int phase = (shootState - 1) % 4;

        if (ringIndex >= ballOrder.length) {
            outtake.disableFlywheel();
            shootState = -1;
            pathState = 5;
            return;
        }

        if (phase == 0) {
            timer = System.currentTimeMillis();
            shootState++;
            return;
        }

        if (phase == 1) {
            if (System.currentTimeMillis() - timer >= FEED_TO_SHOOT_DELAY) {
                timer = System.currentTimeMillis();
                shootState++;
            }
            return;
        }

        if (phase == 2) {
            if (System.currentTimeMillis() - timer >= ARM_UP_TIME) {
                timer = System.currentTimeMillis();
                shootState++;
            }
            return;
        }

        if (phase == 3) {
            if (System.currentTimeMillis() - timer >= POST_DOWN_DELAY) {
                shootState++;
            }
        }
    }

    // ================= PATHS =================
    public static class Paths {
        public PathChain toSee, toShoot, toPark;

        public Paths(Follower follower) {
            // Step 1: forward ~5
            toSee = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(51, 123),
                            new Pose(52, 124)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(87),
                            Math.toRadians(85))
                    .build();

            // Step 2: move slightly left/right while rotating to 120° (~1in left, 1in forward)
            toShoot = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(51, 125),
                            new Pose(52, 126))) // slight move
                    .setLinearHeadingInterpolation(
                            Math.toRadians(85),
                            Math.toRadians(118))
                    .build();

            // Step 3: park forward +16, face 90°
            toPark = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(51, 126),
                            new Pose(52, 145))) // forward
                    .setLinearHeadingInterpolation(
                            Math.toRadians(118),
                            Math.toRadians(90))
                    .build();
        }
    }
}
