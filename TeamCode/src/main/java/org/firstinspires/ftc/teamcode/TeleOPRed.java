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

    // ===== DPAD RISING EDGE =====
    private boolean prevDpadRight = false;
    private boolean prevDpadLeft  = false;
    private boolean prevDpadDown  = false;
    private boolean prevDpadUp = false;

    @Override
    public void init() {
        drive = new Mecnum(); drive.init(hardwareMap);
        intake = new Intake(); intake.init(hardwareMap);
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

        // ===== INTAKE =====
        if (gamepad1.a) intake.intakeForwards();
        else if (gamepad1.b) intake.intakeReverse();
        else intake.stop();
        // ===== LIFT TOGGLE =====
        if (gamepad1.dpad_up && !prevDpadUp) lifts.liftsToggle();
        prevDpadUp = gamepad1.dpad_up;

        prevDpadRight = gamepad1.dpad_right;
        prevDpadLeft  = gamepad1.dpad_left;
        prevDpadDown  = gamepad1.dpad_down;


        // ===== FLYWHEEL CONTROL =====
        if (gamepad1.right_trigger > 0.1) shooter.enableFlywheel();
        else if (gamepad1.left_trigger > 0.1) shooter.disableFlywheel();
        shooter.updateFlywheel();

        // ===== KICKER =====
        if (shootCase == -1){
            if (gamepad1.y) forceShoot();
        } else {
            updateShooting();
        }

        // ===== TELEMETRY =====
        telemetry.addData("Slots", slotOccupied[0] + "," + slotOccupied[1] + "," + slotOccupied[2]);
        telemetry.addData("Colors", slotColor[0] + "," + slotColor[1] + "," + slotColor[2]);
        telemetry.addData("ShootCase", shootCase);
        telemetry.addData("SelectedSlot", rapidNextIndex + 1);
        telemetry.addData("Lift Down", lifts.isDown());
        telemetry.update();
    }


    // private void shootColor(String color) {
    //     for (int i = 0; i < 3; i++) {
    //         if (slotOccupied[i] && slotColor[i].equals(color)) {
    //             rapidNextIndex = i;
    //             shootCase = 0;
    //             shootTimer = System.currentTimeMillis();
    //             singleShot = true;
    //             break;
    //         }
    //     }
    // }

    private void forceShoot() {
        shootCase = 0;
        shootTimer = System.currentTimeMillis();
    }

    private void updateShooting() {
        switch (shootCase) {
            case 0: // Arm Up
                shooter.shooterArmUp();
                if (System.currentTimeMillis() - shootTimer > ARM_UP) {
                    shootCase = 1;
                    shootTimer = System.currentTimeMillis();
                }
                break;
            case 1: // Arm Down
                shooter.shooterArmDown();
                if (System.currentTimeMillis() - shootTimer > ARM_DOWN) {
                    shootCase = -1;
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
        shooter.stop();
        turret.stop();
        lifts.stop();
    }
}