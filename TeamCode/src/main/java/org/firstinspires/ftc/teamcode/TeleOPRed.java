package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subSystems.*;

@Configurable
@TeleOp(name = "TeleOpRed", group = "Linear OpMode")
public class TeleOPRed extends OpMode {

    private Mecnum drive;
    private Intake intake;
    private Spindexer spindexer;
    private Outtake shooter;
    private Turret turret;
    private Vision vision;
    private CSensor color;

    @Override
    public void init() {
        drive = new Mecnum(); drive.init(hardwareMap);
        intake = new Intake(); intake.init(hardwareMap);
        spindexer = new Spindexer(); spindexer.init(hardwareMap);
        shooter = new Outtake(); shooter.init(hardwareMap);
        turret = new Turret(); turret.init(hardwareMap);
        vision = new Vision(); vision.init(hardwareMap); vision.setTargetId(24);
        color = new CSensor(); color.init(hardwareMap);
    }

    @Override
    public void loop() {
        vision.update();
        drive.driveRobot(gamepad1);

        turret.update(vision.getTX());

        // Intake control
        if (gamepad1.a) intake.intakeForwards();
        else if (gamepad1.b) intake.intakeReverse();
        else intake.stop();

        // Shooter Arm control
        if (spindexer.isInOuttake()) {
            if (gamepad1.y) shooter.shooterArmUp();
            else shooter.shooterArmDown();
        } else {
            shooter.shooterArmDown();
        }

        // Spindexer control
        if (shooter.isArmDown()) {
            if (gamepad2.a) spindexer.moveToHold(1);
            if (gamepad2.x) spindexer.moveToHold(2);
            if (gamepad2.b) spindexer.moveToHold(3);
            if (gamepad2.dpad_down) spindexer.moveHoldToOuttake(1);
            if (gamepad2.dpad_left) spindexer.moveHoldToOuttake(2);
            if (gamepad2.dpad_right) spindexer.moveHoldToOuttake(3);
        }

        if (gamepad1.dpad_down) shooter.manualHoodUp();
        else if (gamepad1.dpad_up) shooter.manualHoodDown();

        // Flywheel toggle
        if (gamepad1.right_trigger > 0.1) {
            double distance = 0.6 * vision.getDistanceArea() + 0.4 * vision.getDistanceTrig();
            shooter.setTargetRPM(computeRPMForDistance(distance));
            shooter.enableFlywheel();
        } else if (gamepad1.left_trigger > 0.1) {
            shooter.disableFlywheel();
        }

        shooter.updateFlywheel();

        // Telemetry
        telemetry.addLine("Outtake");
        shooter.addTelemetry(telemetry);
        telemetry.addLine("Spindexer Pos: " + spindexer.getCurrentPosition());
        telemetry.addLine("Turret");
        turret.addTelemetry(telemetry);
        telemetry.update();
    }

    private double computeRPMForDistance(double distance) {
        return 650 + 13.8 * distance; // tune this linear mapping
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