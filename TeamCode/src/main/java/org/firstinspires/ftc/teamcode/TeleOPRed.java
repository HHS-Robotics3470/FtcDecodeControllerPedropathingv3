package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subSystems.*;

@Configurable
@TeleOp(name="TeleOpRed", group="Linear OpMode")
public class TeleOPRed extends OpMode {

    private Mecnum drive;
    private Intake intake;
    private Spindexer spindexer;
    private Outtake shooter;
    private Turret turret;
    private Vision vision;
    private CSensor colorSensors;
    private Lifts lifts;

    // ===== SHOOTING STATE =====
    private int shootCase = -1;
    private long shootTimer = 0;
    private int rapidNextIndex = 0;
    private boolean singleShot = false;
    private static final long ARM_UP = 250;
    private static final long ARM_DOWN = 250;

    // ===== SLOT TRACKING =====
    private boolean[] slotOccupied = {false, false, false};
    private String[] slotColor = {"None", "None", "None"};

    // ===== AUTO SPINDEXER =====
    private boolean ballDebouncing = false;
    private long ballDebounceTimer = 0;
    private static final long BALL_DEBOUNCE = 300;
    private boolean manualSpindexOverride = false;

    // ===== DPAD RISING EDGE =====
    private boolean prevDpadRight = false;
    private boolean prevDpadLeft  = false;
    private boolean prevDpadDown  = false;
    private boolean prevDpadUp    = false;

    @Override
    public void init() {
        drive = new Mecnum(); drive.init(hardwareMap);
        intake = new Intake(); intake.init(hardwareMap);
        spindexer = new Spindexer(); spindexer.init(hardwareMap);
        shooter = new Outtake(); shooter.init(hardwareMap);
        turret = new Turret(); turret.init(hardwareMap);
        vision = new Vision(); vision.init(hardwareMap);
        vision.setValidIds(20);
        colorSensors = new CSensor(); colorSensors.init(hardwareMap);
        lifts = new Lifts(); lifts.init(hardwareMap);
    }

    @Override
    public void loop() {
        drive.driveRobot(gamepad1);
        turret.update(vision.getTX());

        // ===== SENSOR SLOT SYNC =====
        for (int i = 0; i < 3; i++) {
            slotOccupied[i] = colorSensors.ballPresent(i);
            slotColor[i] = colorSensors.getColor(i);
        }

        boolean allFull = slotOccupied[0] && slotOccupied[1] && slotOccupied[2];

        // ===== INTAKE =====
        if (!allFull) {
            if (gamepad1.a) intake.intakeForwards();
            else if (gamepad1.b) intake.intakeReverse();
            else intake.stop();
        } else {
            intake.stop();
        }

        // ===== LIFT TOGGLE =====
        if (gamepad1.dpad_up && !prevDpadUp) lifts.liftsToggle();
        prevDpadUp = gamepad1.dpad_up;

        // ===== MANUAL SPINDEXER ROTATION =====
        boolean manualPressed = gamepad2.a || gamepad2.b || gamepad2.x
                || gamepad1.dpad_right || gamepad1.dpad_left || gamepad1.dpad_down;

        if (gamepad2.a) { rotateToSlot(1); manualSpindexOverride = true; }
        if (gamepad2.b) { rotateToSlot(2); manualSpindexOverride = true; }
        if (gamepad2.x) { rotateToSlot(3); manualSpindexOverride = true; }

        if (gamepad1.dpad_right && !prevDpadRight) { rotateToSlot(1); manualSpindexOverride = true; }
        if (gamepad1.dpad_left  && !prevDpadLeft)  { rotateToSlot(2); manualSpindexOverride = true; }
        if (gamepad1.dpad_down  && !prevDpadDown)  { rotateToSlot(3); manualSpindexOverride = true; }

        if (!manualPressed) manualSpindexOverride = false;

        prevDpadRight = gamepad1.dpad_right;
        prevDpadLeft  = gamepad1.dpad_left;
        prevDpadDown  = gamepad1.dpad_down;

        // ===== AUTO SPINDEXER =====
        if (!manualSpindexOverride && shootCase == -1) {
            boolean ballAtSlot0 = colorSensors.ballPresent(0);

            if (ballAtSlot0 && (!slotOccupied[1] || !slotOccupied[2])) {
                if (!ballDebouncing) {
                    ballDebouncing = true;
                    ballDebounceTimer = System.currentTimeMillis();
                }
            } else {
                ballDebouncing = false;
            }

            if (ballDebouncing && System.currentTimeMillis() - ballDebounceTimer >= BALL_DEBOUNCE) {
                ballDebouncing = false;
                int nextEmpty = firstEmptySlotAfter(0);
                if (nextEmpty != -1) {
                    rotateToSlot(nextEmpty);
                }
            }
        }

        // ===== SHOOTING =====
        if (shootCase == -1) {
            if (gamepad2.y) shootSlot(rapidNextIndex);
            if (gamepad2.dpad_up) startShootAll();
        }

        runShootCases();

        // ===== SPINDEXER UPDATE =====
        spindexer.update();

        // ===== FLYWHEEL CONTROL =====
        if (gamepad1.right_trigger > 0.1) shooter.enableFlywheel();
        else if (gamepad1.left_trigger > 0.1) shooter.disableFlywheel();
        shooter.updateFlywheel();

        // ===== KICKER =====
        if (shootCase == -1) {
            if (gamepad1.y) forceShoot();
        }

        // ===== TELEMETRY =====
        telemetry.addData("Slots", slotOccupied[0] + "," + slotOccupied[1] + "," + slotOccupied[2]);
        telemetry.addData("Colors", slotColor[0] + "," + slotColor[1] + "," + slotColor[2]);
        telemetry.addData("ShootCase", shootCase);
        telemetry.addData("SelectedSlot", rapidNextIndex + 1);
        telemetry.addData("Spindexer Pos", spindexer.getCurrentPosition());
        telemetry.addData("Spindexer Target", spindexer.getTargetPosition());
        telemetry.addData("Spindexer At Target", spindexer.atTarget());
        telemetry.addData("Spindexer Mode", spindexer.getMode());
        telemetry.addData("Spindexer Active", spindexer.getActive());
        telemetry.addData("Spindexer Error", spindexer.getTargetPosition() - spindexer.getCurrentPosition());
        telemetry.addData("Lift Down", lifts.isDown());
        telemetry.addData("All Slots Full", allFull);
        telemetry.addData("Ball Debouncing", ballDebouncing);
        telemetry.update();

    }

    private int firstEmptySlot() {
        for (int i = 0; i < 3; i++) if (!slotOccupied[i]) return i + 1;
        return -1;
    }

    private int firstEmptySlotAfter(int after) {
        for (int i = after + 1; i < 3; i++) if (!slotOccupied[i]) return i + 1;
        return -1;
    }

    private void forceShoot() {
        shootCase = 0;
        shootTimer = System.currentTimeMillis();
        singleShot = true;
    }

    private void rotateToSlot(int slot) {
        if (slot >= 1 && slot <= 3) {
            rapidNextIndex = slot - 1;
            spindexer.moveToSlot(slot);
        }
    }

    private void shootSlot(int slotIndex) {
        if (slotIndex >= 0 && slotIndex < 3 && slotOccupied[slotIndex]) {
            rapidNextIndex = slotIndex;
            shootCase = 0;
            shootTimer = System.currentTimeMillis();
            singleShot = true;
        }
    }

    private void startShootAll() {
        for (int i = 0; i < 3; i++) {
            if (slotOccupied[i]) {
                rapidNextIndex = i;
                shootCase = 0;
                shootTimer = System.currentTimeMillis();
                singleShot = false;
                break;
            }
        }
    }

    private void runShootCases() {
        if (shootCase == -1) return;

        long now = System.currentTimeMillis();
        switch (shootCase) {
            case 0:
                shooter.shooterArmUp();
                shootTimer = now;
                shootCase = 1;
                break;

            case 1:
                if (now - shootTimer >= ARM_UP) {
                    shooter.shooterArmDown();
                    shootTimer = now;
                    shootCase = 2;
                }
                break;

            case 2:
                if (now - shootTimer >= ARM_DOWN) {
                    shootCase = 3;
                }
                break;

            case 3:
                if (spindexer.atTarget()) {
                    slotOccupied[rapidNextIndex] = false;
                    slotColor[rapidNextIndex] = "None";

                    if (!singleShot) {
                        int next = nextOccupiedSlot(rapidNextIndex);
                        if (next != -1) {
                            rapidNextIndex = next;
                            shootCase = 0;
                        } else shootCase = -1;
                    } else shootCase = -1;
                }
                break;
        }
    }

    private int nextOccupiedSlot(int current) {
        for (int i = current + 1; i < 3; i++) if (slotOccupied[i]) return i;
        for (int i = 0; i <= current; i++) if (slotOccupied[i]) return i;
        return -1;
    }

    @Override
    public void stop() {
        drive.stop();
        intake.stop();
        spindexer.stop();
        shooter.stop();
        turret.stop();
        lifts.stop();
    }
}