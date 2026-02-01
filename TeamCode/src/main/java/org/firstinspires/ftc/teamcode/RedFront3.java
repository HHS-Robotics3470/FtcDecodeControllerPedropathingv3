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
@Autonomous(name = "Red Front 3", group = "Autonomous")
public class RedFront3 extends OpMode {

    private Vision vision;
    private Outtake outtake;
    private Spindexer spindexer;
    private Follower follower;
    private Paths paths;

    private int pathState = 0;
    private int shootState = -1;
    private int patternId = -1;
    private long timer = 0;

    // ===== TIMING CONSTANTS =====

    private static final long FLYWHEEL_SPINUP_TIME = 4000;
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

        follower = Constants.createFollower(hardwareMap);
        // RED start pose (mirrored from Blue)
        follower.setStartingPose(new Pose(124, 123, Math.toRadians(36)));

        paths = new Paths(follower);

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        vision.updateVision();
        follower.update();

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

        // INIT
        if (shootState == -1) {
            outtake.autoShootPreload();     // flywheel ON
            outtake.autoShooterArmDown();   // lever starts DOWN
            timer = System.currentTimeMillis();
            shootState = 0;
            return;
        }

        // FLYWHEEL SPIN-UP
        if (shootState == 0) {
            if (System.currentTimeMillis() - timer >= FLYWHEEL_SPINUP_TIME) {
                shootState = 1;
                timer = System.currentTimeMillis();
            }
            return;
        }

        int ringIndex = (shootState - 1) / 4;
        int phase     = (shootState - 1) % 4;

        if (ringIndex >= ballOrder.length) {
            outtake.disableFlywheel();
            spindexer.moveToHold(1);
            shootState = -1;
            pathState = 5;
            return;
        }

        // PHASE 0: FEED RING
        if (phase == 0) {
            spindexer.moveHoldToOuttake(ballOrder[ringIndex]);
            timer = System.currentTimeMillis();
            shootState++;
            return;
        }

        // PHASE 1: ARM UP (SHOOT)
        if (phase == 1) {
            if (System.currentTimeMillis() - timer >= FEED_TO_SHOOT_DELAY) {
                outtake.autoShooterArmUp();
                timer = System.currentTimeMillis();
                shootState++;
            }
            return;
        }

        // PHASE 2: ARM DOWN
        if (phase == 2) {
            if (System.currentTimeMillis() - timer >= ARM_UP_TIME) {
                outtake.autoShooterArmDown();
                timer = System.currentTimeMillis();
                shootState++;
            }
            return;
        }

        // PHASE 3: POST-DOWN BREAK
        if (phase == 3) {
            if (System.currentTimeMillis() - timer >= POST_DOWN_DELAY) {
                timer = System.currentTimeMillis();
                shootState++;
            }
        }
    }

    // ================= PATHS =================
    public static class Paths {
        public PathChain toSee, toShoot, toPark;

        public Paths(Follower follower) {
            // Move from start to see the target
            toSee = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(124, 123),
                            new Pose(88, 84)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(95),
                            Math.toRadians(95))
                    .build();

            // Move to shooting position
            toShoot = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(124, 123),
                            new Pose(88, 84)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(30),
                            Math.toRadians(30))
                    .build();

            // Move to park
            toPark = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(88, 123),
                            new Pose(88, 65)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(90),  // face top of field
                            Math.toRadians(90))
                    .build();
        }
    }


}
