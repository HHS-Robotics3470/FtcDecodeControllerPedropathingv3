package org.firstinspires.ftc.teamcode;

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

@Autonomous(name = "Blue Front 3", group = "Autonomous")
public class BlueFront3 extends OpMode {

    private Vision vision;
    private Outtake outtake;
    private Spindexer spindexer;
    private Follower follower;
    private Paths paths;

    private int pathState = 0;
    private int shootState = -1;
    private int patternId = -1;
    private long timer = 0;

    @Override
    public void init() {
        vision = new Vision();
        vision.init(hardwareMap);
        vision.setValidIds(21, 22, 23); // Only detect pattern IDs

        outtake = new Outtake(); outtake.init(hardwareMap);
        spindexer = new Spindexer(); spindexer.init(hardwareMap);

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

        if (pathState == 4 && patternId != -1) {
            switch (patternId) {
                case 21: runShootingSequence(new int[]{2,1,3}); break;
                case 22: runShootingSequence(new int[]{1,2,3}); break;
                case 23: runShootingSequence(new int[]{1,3,2}); break;
            }
        }

        telemetry.addData("Pattern ID", patternId);
        telemetry.addData("PathState", pathState);
        telemetry.addData("ShootState", shootState);
        telemetry.addData("Distance", vision.getDistance());
        telemetry.update();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0: follower.followPath(paths.toSee); pathState = 1; break;
            case 1: if (!follower.isBusy()) pathState = 2; break;
            case 2: if (patternId != -1) { follower.followPath(paths.toShoot); pathState = 3; } break;
            case 3: if (!follower.isBusy()) pathState = 4; break;
            case 4: break; // wait for shooting FSM
            case 5: follower.followPath(paths.toPark); pathState = 6; break;
            case 6: if (!follower.isBusy()) pathState = 7; break;
            case 7: break;
        }
    }

    private void runShootingSequence(int[] ballOrder) {
        if (shootState == -1) {
            outtake.setDistance(vision.getDistance());
            outtake.enableFlywheel();
            timer = System.currentTimeMillis();
            shootState = 0;
        }

        if (shootState >= 0 && shootState < ballOrder.length) {
            if (System.currentTimeMillis() - timer > 1500) {
                spindexer.moveHoldToOuttake(ballOrder[shootState]);
                timer = System.currentTimeMillis();
                shootState++;
            }
        } else if (shootState == ballOrder.length) {
            if (System.currentTimeMillis() - timer > 1000) {
                shootState = -1;
                pathState = 5;
                spindexer.moveToHold(1);
                outtake.disableFlywheel();
            }
        }
    }

    public static class Paths {
        public PathChain toSee, toShoot, toPark;

        public Paths(Follower follower) {
            toSee = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(21, 123), new Pose(56, 84)))
                    .setLinearHeadingInterpolation(Math.toRadians(75), Math.toRadians(75)).build();

            toShoot = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(21, 123), new Pose(56, 84)))
                    .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(144)).build();

            toPark = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(56, 123), new Pose(56, 84)))
                    .setLinearHeadingInterpolation(Math.toRadians(75), Math.toRadians(75)).build();
        }
    }
}
