package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subSystems.Intake;
import org.firstinspires.ftc.teamcode.subSystems.Outtake;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class BlueBack3 extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower;                 // Pedro Pathing follower instance
    private int pathState;                    // Current autonomous state
    private Paths paths;                      // Paths defined below
    private final ElapsedTime stateTimer = new ElapsedTime();

    private Intake intake;
    private Outtake shooter; // named "shooter" to match TeleOPRed's convention

    // Timing constants — matched to TeleOPRed's kicker cycle (ARM_UP / ARM_DOWN = 250ms)
    private static final long WAIT_BEFORE_PATH3_MS = 300; // matches the Wait block in the visualizer
    private static final long FLYWHEEL_SPINUP_MS = 500;   // time to let the flywheel get up to speed
    private static final long ARM_UP_MS = 250;
    private static final long ARM_DOWN_MS = 250;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        // Starting pose matches the start of Path 1 (82, 10), heading 90 to match Path 1's start heading.
        follower.setStartingPose(new Pose(82, 10, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        intake = new Intake();
        intake.init(hardwareMap);

        shooter = new Outtake();
        shooter.init(hardwareMap);

        pathState = 0;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        stateTimer.reset();
        follower.followPath(paths.path1);
        pathState = 0;
    }

    @Override
    public void loop() {
        follower.update();       // Update Pedro Pathing
        shooter.updateFlywheel(); // Must be called every loop — applies power based on enable/disable flag
        autonomousPathUpdate();  // Advance the state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Busy", follower.isBusy());
        panelsTelemetry.update(telemetry);
    }

    /**
     * State machine:
     * Path 1 -> Path 2 -> [intake on] -> 300ms wait -> Path 3 -> [flywheel spin-up]
     * -> kicker up (shoot) -> kicker down -> flywheel off -> done.
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // (82,10) -> (58,36), heading 90 -> 0
                if (!follower.isBusy()) {
                    follower.followPath(paths.path2);
                    setPathState(1);
                }
                break;

            case 1: // (58,36) -> (82,36), tangential heading
                if (!follower.isBusy()) {
                    // End of Path 2: start the intake
                    intake.intakeForwards();
                    setPathState(2);
                }
                break;

            case 2: // 300ms wait before starting Path 3 (matches the Wait block in the visualizer)
                if (stateTimer.milliseconds() > WAIT_BEFORE_PATH3_MS) {
                    follower.followPath(paths.path3);
                    setPathState(3);
                }
                break;

            case 3: // (82,36) -> (36,36), heading 0 -> 35
                if (!follower.isBusy()) {
                    // End of Path 3: stop intake, spin up the outtake shooter (flywheel)
                    intake.stop();
                    shooter.enableFlywheel();
                    setPathState(4);
                }
                break;

            case 4: // Let the flywheel spin up to speed before feeding a ball
                if (stateTimer.milliseconds() > FLYWHEEL_SPINUP_MS) {
                    shooter.shooterArmUp(); // kicker up — feeds the ball into the flywheel
                    setPathState(5);
                }
                break;

            case 5: // Hold kicker up, then bring it back down
                if (stateTimer.milliseconds() > ARM_UP_MS) {
                    shooter.shooterArmDown();
                    setPathState(6);
                }
                break;

            case 6: // Hold kicker down, then shut off the flywheel and finish
                if (stateTimer.milliseconds() > ARM_DOWN_MS) {
                    shooter.disableFlywheel();
                    setPathState(-1);
                }
                break;

            case -1: // Done — idle
                break;
        }
    }

    private void setPathState(int state) {
        pathState = state;
        stateTimer.reset();
    }

    @Override
    public void stop() {
        intake.stop();
        shooter.stop();
    }

    public static class Paths {
        public PathChain path1, path2, path3;

        public Paths(Follower follower) {
            path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(82.000, 10.000),
                            new Pose(58.000, 36.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            path2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(58.000, 36.000),
                            new Pose(82.000, 36.000)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            path3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(82.000, 36.000),
                            new Pose(36.000, 36.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(35))
                    .build();
        }
    }
}