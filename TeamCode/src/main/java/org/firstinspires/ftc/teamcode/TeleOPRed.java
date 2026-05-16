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

    // Shooting
    private int shootCase = -1;
    private long shootTimer = 0;
    private int rapidNextIndex = 0;
    private boolean singleShot = false;
    private static final long ARM_UP = 250;
    private static final long ARM_DOWN = 250;

    // Ball Slots
    private boolean[] slotOccupied = {false, false, false};
    private String[] slotColor = {"None", "None", "None"};

    // Edge Detection
    private boolean lastDpadDown = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadLeft = false;

    @Override
    public void init() {
        drive = new Mecnum(); drive.init(hardwareMap);
        intake = new Intake(); intake.init(hardwareMap);
        spindexer = new Spindexer(); spindexer.init(hardwareMap);
        shooter = new Outtake(); shooter.init(hardwareMap);
        turret = new Turret(); turret.init(hardwareMap);
        vision = new Vision(); vision.init(hardwareMap);
        vision.setValidIds(20);

        colorSensors = new CSensor();
        colorSensors.init(hardwareMap);
    }

    @Override
    public void loop() {
        drive.driveRobot(gamepad1);
        turret.update(vision.getTX());

        // ===== INTAKE =====
        if (gamepad1.a) intake.intakeForwards();
        else if (gamepad1.b) intake.intakeReverse();
        else intake.stop();

        // Manual Spindexer Rotation
        if (gamepad1.dpad_down && !lastDpadDown) rotateToSlot(2);
        if (gamepad1.dpad_right && !lastDpadRight) rotateToSlot(3);
        if (gamepad1.dpad_left && !lastDpadLeft) rotateToSlot(1);

        lastDpadDown = gamepad1.dpad_down;
        lastDpadRight = gamepad1.dpad_right;
        lastDpadLeft = gamepad1.dpad_left;

        // Flywheel Control
        if (gamepad1.right_trigger > 0.1) shooter.enableFlywheel();
        else if (gamepad1.left_trigger > 0.1) shooter.disableFlywheel();
        shooter.updateFlywheel();

        // yayy Shooting
        if (shootCase == -1) {
            if (gamepad1.y) forceShoot();
            if (gamepad2.dpad_left) shootColor("Green");
            if (gamepad2.dpad_right) shootColor("Purple");
            if (gamepad2.dpad_up) startShootAll();
        }

        runShootCases();

        // Update the Spindexer or smth
        spindexer.update();

        // Telemetry Stuff
        telemetry.addData("Slots", slotOccupied[0] + "," + slotOccupied[1] + "," + slotOccupied[2]);
        telemetry.addData("Colors", slotColor[0] + "," + slotColor[1] + "," + slotColor[2]);
        telemetry.addData("ShootCase", shootCase);
        telemetry.addData("SelectedSlot", rapidNextIndex + 1);
        // ===== DEBUG - tells us what spindexer is doing =====
        telemetry.addData("Spindexer Pos", spindexer.getPosition());
        telemetry.addData("Spindexer Target", spindexer.getTarget());
        telemetry.addData("Spindexer Moving", spindexer.getIsMoving());
        telemetry.update();
    }

    // Shoots immediately without checking slot state
    private void forceShoot() {
        shootCase = 0;
        shootTimer = System.currentTimeMillis();
        singleShot = true;
    }

    private int firstEmptySlot() {
        for (int i = 0; i < 3; i++) if (!slotOccupied[i]) return i + 1;
        return -1;
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

    private void shootColor(String color) {
        for (int i = 0; i < 3; i++) {
            if (slotOccupied[i] && slotColor[i].equals(color)) {
                rapidNextIndex = i;
                shootCase = 0;
                shootTimer = System.currentTimeMillis();
                singleShot = true;
                break;
            }
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
            case 0: // arm up
                shooter.shooterArmUp();
                shootTimer = now;
                shootCase = 1;
                break;

            case 1: // arm up pause
                if (now - shootTimer >= ARM_UP) {
                    shooter.shooterArmDown();
                    shootTimer = now;
                    shootCase = 2;
                }
                break;

            case 2: // arm down -> done
                if (now - shootTimer >= ARM_DOWN) {
                    shootCase = -1;
                }
                break;

            case 3: // wait for spindexer to reach target
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
    }
}