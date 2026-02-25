package org.firstinspires.ftc.teamcode.subSystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class Outtake implements Subsystems {

    private DcMotorEx flywheelMotor1;
    private Servo hoodServo;
    private Servo shootServo;

    // ===== Tunables =====
    public static double HOOD_MIN = 0.0;
    public static double HOOD_MAX = 0.55;

    public static double SHOOTER_ARM_DOWN = 0.97;
    public static double SHOOTER_ARM_UP = 0.6;

    public static double MOTOR_POWER = 1.0;

    private boolean flywheelOn = false;
    private boolean armDown = true;

    @Override
    public void init(HardwareMap hw) {

        flywheelMotor1 = hw.get(DcMotorEx.class, "flywheelMotor1");
        flywheelMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        hoodServo = hw.get(Servo.class, "hoodServo");
        shootServo = hw.get(Servo.class, "shootServo");

        hoodServo.setPosition(HOOD_MIN);
        shootServo.setPosition(SHOOTER_ARM_DOWN);
    }

    // ================================
    // Shooter Arm
    // ================================
    public void shooterArmUp() {
        shootServo.setPosition(SHOOTER_ARM_UP);
        armDown = false;
    }

    public void shooterArmDown() {
        shootServo.setPosition(SHOOTER_ARM_DOWN);
        armDown = true;
    }

    public boolean isArmDown() {
        return armDown;
    }

    // ================================
    // Flywheel Control
    // ================================
    public void enableFlywheel() {
        flywheelOn = true;
    }

    public void disableFlywheel() {
        flywheelOn = false;
        flywheelMotor1.setPower(0);
    }

    public void updateFlywheel() {
        if (!flywheelOn) {
            flywheelMotor1.setPower(0);
            return;
        }
        flywheelMotor1.setPower(MOTOR_POWER);
    }

    public boolean isFlywheelOn() {
        return flywheelOn;
    }

    // ================================
    // Hood Control (Optional)
    // ================================
    public void setHoodPosition(double pos) {
        pos = Math.max(HOOD_MIN, Math.min(HOOD_MAX, pos));
        hoodServo.setPosition(pos);
    }

    public double getHoodPosition() {
        return hoodServo.getPosition();
    }

    // ================================
    // Telemetry
    // ================================
    public void addTelemetry(Telemetry t) {
        t.addData("Flywheel On", flywheelOn);
        t.addData("Flywheel Power", MOTOR_POWER);
        t.addData("Hood Pos", hoodServo.getPosition());
        t.addData("Arm Down", armDown);
    }

    @Override
    public void stop() {
        disableFlywheel();
        hoodServo.setPosition(HOOD_MIN);
        shootServo.setPosition(SHOOTER_ARM_DOWN);
    }
}