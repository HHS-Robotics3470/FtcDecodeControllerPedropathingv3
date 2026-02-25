package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subSystems.Outtake;
import org.firstinspires.ftc.teamcode.subSystems.Spindexer;
import org.firstinspires.ftc.teamcode.subSystems.Vision;
import org.firstinspires.ftc.teamcode.subSystems.Intake;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Front 6", group = "Autonomous")
public class BlueFront6 extends OpMode {

    private Vision vision;
    private Outtake outtake;
    private Spindexer spindexer;
    private Intake intake;
    private Follower follower;
    private Paths paths;

    private int pathState = 0;
    private int shootState = -1;
    private int patternId = -1;
    private long timer = 0;

    private boolean preloadShotDone = false;
    private boolean secondShotDone = false;

    // ================= SPEED CONTROL =================
    private static final double NORMAL_SPEED = 1.0;
    private static final double INTAKE_SPEED = 0.55;

    // ================= SHOOT TIMING =================
    private static final long FLYWHEEL_SPINUP_TIME = 5000;
    private static final long FEED_TO_SHOOT_DELAY = 600;
    private static final long ARM_UP_TIME = 150;
    private static final long POST_DOWN_DELAY = 0;

    private long intakeTimer = 0;

    @Override
    public void init() {

        vision = new Vision();
        vision.init(hardwareMap);
        vision.setValidIds(21, 22, 23);

        outtake = new Outtake();
        outtake.init(hardwareMap);

        spindexer = new Spindexer();
        spindexer.init(hardwareMap);

        intake = new Intake();
        intake.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(21, 123, Math.toRadians(144)));

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

        telemetry.addData("Pattern ID", patternId);
        telemetry.addData("PathState", pathState);
        telemetry.update();
    }

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
                if (!preloadShotDone) {
                    runShootingSequence(new int[]{1,2,3});
                }
                break;

            case 5:
                follower.followPath(paths.toPickupPrep);
                pathState = 6;
                break;

            case 6: // START SWEEP PICKUP
                follower.setMaxPower(INTAKE_SPEED);
                intake.intakeForwards();
                intakeTimer = System.currentTimeMillis();
                follower.followPath(paths.pickupSweep);
                pathState = 7;
                break;

            case 7: // SWEEP INTAKE
                intake.intakeForwards();

                long elapsed = System.currentTimeMillis() - intakeTimer;

                if (elapsed > 600 && elapsed < 800) {
                }
                if (elapsed > 1200 && elapsed < 1400) {
                }
                if (elapsed > 1800 && elapsed < 2000) {
                }

                if (!follower.isBusy()) {
                    intake.stop();
                    follower.setMaxPower(NORMAL_SPEED);
                    follower.followPath(paths.backToShoot);
                    pathState = 8;
                }
                break;

            case 8:
                if (!follower.isBusy()) pathState = 9;
                break;

            case 9:
                if (!secondShotDone) {
                    runShootingSequence(getPatternArray());
                }
                break;

            case 10:
                follower.followPath(paths.toPark);
                pathState = 11;
                break;

            case 11:
                break;
        }
    }

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

            if (!preloadShotDone) {
                preloadShotDone = true;
                pathState = 5;
            } else {
                secondShotDone = true;
                pathState = 10;
            }
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
                timer = System.currentTimeMillis();
                shootState++;
            }
        }
    }

    private int[] getPatternArray() {
        switch (patternId) {
            case 21: return new int[]{2,1,3};
            case 22: return new int[]{1,2,3};
            case 23: return new int[]{1,3,2};
        }
        return new int[]{1,2,3};
    }

    public static class Paths {

        public PathChain toSee, toShoot, toPickupPrep, pickupSweep, backToShoot, toPark;

        public Paths(Follower follower) {

            toSee = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(21, 123), new Pose(56, 84)))
                    .setLinearHeadingInterpolation(Math.toRadians(85), Math.toRadians(85))
                    .build();

            toShoot = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(21, 123), new Pose(56, 84)))
                    .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(150))
                    .build();

            toPickupPrep = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(56, 84), new Pose(42, 81)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            pickupSweep = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(42, 81), new Pose(27, 81)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            backToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(27, 81), new Pose(56, 84)))
                    .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(150))
                    .build();

            toPark = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(56, 84), new Pose(56, 65)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();
        }
    }
}