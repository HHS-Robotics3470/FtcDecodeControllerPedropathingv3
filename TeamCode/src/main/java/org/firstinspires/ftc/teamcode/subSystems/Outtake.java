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
    public static double HOOD_MIN = 0.0;
    public static double HOOD_MAX = 0.55;
    public static double hoodManualStep = 0.01;

    public static double SHOOTER_ARM_DOWN = 0.0;
    public static double SHOOTER_ARM_UP = 0.25;

    public static double MOTOR_POWER = 0.0;

    private boolean flywheelOn = false;
    private boolean armDown = true;
    private final double[] distanceValues = {
            36, 40, 44, 48, 52, 60, 66, 72,
            78, 82, 96, 102, 108, 120, 126, 132, 141
    };
    private final double[] powerValues = {
            0.63, 0.64, 0.65, 0.655, 0.66, 0.68, 0.69, 0.75,
            0.77, 0.775, 0.76, 0.778, 0.78, 0.79, 0.83, 0.865, 0.90
    };
    private final double[] hoodDistanceValues = {
            36, 40, 44, 52, 60, 66, 72, 82,
            96, 102, 108, 120, 126, 132, 141
    };
    private final double[] hoodPositionValues = {
            0.0, 0.0, 0.0, 0.0,
            0.09, 0.20, 0.30, 0.48,
            0.50, 0.55, 0.55, 0.55,
            0.55, 0.55, 0.55
    };
    @Override
    public void init(HardwareMap hardwareMap) {
        flywheelMotor1 = hardwareMap.get(DcMotorEx.class, "flywheelMotor1");
        flywheelMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "flywheelMotor2");
        flywheelMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        shootServo = hardwareMap.get(Servo.class, "shootServo");

        hoodServo.setPosition(HOOD_MIN);
        shootServo.setPosition(SHOOTER_ARM_DOWN);
    }

    // ===== SHOOTER ARM =====
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

    // ===== MANUAL HOOD =====
    public void manualHoodUp() {
        hoodServo.setPosition(
                Math.min(hoodServo.getPosition() + hoodManualStep, HOOD_MAX)
        );
    }

    public void manualHoodDown() {
        hoodServo.setPosition(
                Math.max(hoodServo.getPosition() - hoodManualStep, HOOD_MIN)
        );
    }

    // ===== INTERPOLATED FLYWHEEL POWER =====
    public double calculateShooterPower(double distance) {
        if (distance <= 0) return 0;

        for (int i = 0; i < distanceValues.length - 1; i++) {
            if (distance >= distanceValues[i] &&
                    distance <= distanceValues[i + 1]) {

                double ratio =
                        (distance - distanceValues[i]) /
                                (distanceValues[i + 1] - distanceValues[i]);

                return powerValues[i] +
                        ratio * (powerValues[i + 1] - powerValues[i]);
            }
        }

        if (distance < distanceValues[0]) return powerValues[0];
        return powerValues[powerValues.length - 1];
    }

    // ===== INTERPOLATED HOOD =====
    public double calculateHoodPosition(double distance) {
        if (distance <= hoodDistanceValues[0])
            return hoodPositionValues[0];

        for (int i = 0; i < hoodDistanceValues.length - 1; i++) {
            if (distance >= hoodDistanceValues[i] &&
                    distance <= hoodDistanceValues[i + 1]) {

                double ratio =
                        (distance - hoodDistanceValues[i]) /
                                (hoodDistanceValues[i + 1] - hoodDistanceValues[i]);

                return hoodPositionValues[i] +
                        ratio * (hoodPositionValues[i + 1] - hoodPositionValues[i]);
            }
        }
        return hoodPositionValues[hoodPositionValues.length - 1];
    }

    // ===== AUTO SET FROM DISTANCE =====
    public void setDistance(double distance) {
        MOTOR_POWER = calculateShooterPower(distance);
        hoodServo.setPosition(calculateHoodPosition(distance));
    }

    // ===== FLYWHEEL =====
    public void enableFlywheel() {
        flywheelOn = true;
    }

    public void disableFlywheel() {
        flywheelOn = false;
        flywheelMotor1.setPower(0);
        flywheelMotor2.setPower(0);
    }

    public void updateFlywheel() {
        if (!flywheelOn) {
            flywheelMotor1.setPower(0);
            flywheelMotor2.setPower(0);
            return;
        }
        flywheelMotor1.setPower(MOTOR_POWER);
        flywheelMotor2.setPower(MOTOR_POWER);
    }

    // ===== TELEMETRY =====
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Flywheel On", flywheelOn);
        telemetry.addData("Motor Power", MOTOR_POWER);
        telemetry.addData("Hood Pos", hoodServo.getPosition());
        telemetry.addData("Arm Down", armDown);
    }

    @Override
    public void stop() {
        disableFlywheel();
        hoodServo.setPosition(HOOD_MIN);
        shootServo.setPosition(SHOOTER_ARM_DOWN);
    }
}
