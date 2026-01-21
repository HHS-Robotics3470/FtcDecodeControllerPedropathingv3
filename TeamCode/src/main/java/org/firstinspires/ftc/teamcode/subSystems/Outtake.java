
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
    private DcMotorEx flywheelMotor2;
    private Servo hoodServo;
    private Servo shootServo;

    // ===== TUNABLES =====
    public static double HOOD_MAX = 0.55;
    public static double HOOD_MIN = 0.0;
    public static double hoodManualStep = 0.01;
    public static double SHOOTER_ARM_DOWN = 0.0;
    public static double SHOOTER_ARM_UP = 0.28;

    private double targetRPM = 0.0;
    private boolean flywheelOn = false;

    public Outtake() {}

    @Override
    public void init(HardwareMap hardwareMap) {
        flywheelMotor1 = hardwareMap.get(DcMotorEx.class, "flywheelMotor1");
        flywheelMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheelMotor1.setPower(0);

        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "flywheelMotor2");
        flywheelMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheelMotor2.setPower(0);

        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        shootServo = hardwareMap.get(Servo.class, "shootServo");

        hoodServo.setPosition(HOOD_MAX);
        shootServo.setPosition(SHOOTER_ARM_DOWN);
    }

    // ===== HOOD =====
    public void manualHoodUp() {
        hoodServo.setPosition(Math.min(HOOD_MAX, hoodServo.getPosition() + hoodManualStep));
    }

    public void manualHoodDown() {
        hoodServo.setPosition(Math.max(HOOD_MIN, hoodServo.getPosition() - hoodManualStep));
    }

    public void Hooddown(){
        hoodServo.setPosition(HOOD_MAX);
    }

    public void Hoodup(){
        hoodServo.setPosition(HOOD_MIN);
    }

    // ===== SHOOTER ARM =====
    public void shooterArmUp() {
        shootServo.setPosition(SHOOTER_ARM_UP);
    }

    public void shooterArmDown() {
        shootServo.setPosition(SHOOTER_ARM_DOWN);
    }

    public boolean isArmDown() {
        return Math.abs(shootServo.getPosition() - SHOOTER_ARM_DOWN) < 0.01;
    }

    // ===== FLYWHEEL =====
    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
    }

    public void enableFlywheel() {
        flywheelOn = true;
    }

    public void disableFlywheel() {
        flywheelOn = false;
        flywheelMotor1.setPower(0);
        flywheelMotor2.setPower(0);
    }

//    public void updateFlywheel() {
//        if (!flywheelOn) {
//            flywheelMotor1.setPower(0);
//            flywheelMotor2.setPower(0);
//            return;
//        }
//        double power = targetRPM / 2000.0; // scale factor, tune for your motor
//        power = Math.max(0.0, Math.min(1.0, power));
//        flywheelMotor1.setPower(power);
//        flywheelMotor2.setPower(power);
//    }

    public void updateFlywheel() {//test
        if (!flywheelOn) {
            flywheelMotor1.setPower(0);
            flywheelMotor2.setPower(0);
            return;
        }
        double power = 1;
        flywheelMotor1.setPower(power);
        flywheelMotor2.setPower(power);
//    }
    }


    // ===== TELEMETRY =====
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Hood Pos", hoodServo.getPosition());
        telemetry.addData("Shooter Arm Pos", shootServo.getPosition());
        telemetry.addData("Flywheel1 Power", flywheelMotor1.getPower());
        telemetry.addData("Flywheel2 Power", flywheelMotor2.getPower());
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Flywheel On", flywheelOn);
    }

    @Override
    public void stop() {
        flywheelMotor1.setPower(0);
        flywheelMotor2.setPower(0);
        shootServo.setPosition(SHOOTER_ARM_DOWN);
    }
    public void shooterOn() {
        flywheelMotor1.setPower(0.67);
        flywheelMotor2.setPower(0.67);
        shootServo.setPosition(SHOOTER_ARM_DOWN);
    }
    public void shooterOnshort() {
        flywheelMotor1.setPower(0.44);
        flywheelMotor2.setPower(0.44);
        shootServo.setPosition(SHOOTER_ARM_DOWN);
    }
}
