package org.firstinspires.ftc.teamcode.subSystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Mecnum implements Subsystems {
    private RobotHardware robotHardware;

    public final double DRIVE_SPEED_MAX = 1;
    public double driveSpeedControl = DRIVE_SPEED_MAX;

    public DcMotorEx fLeft;
    public DcMotorEx fRight;
    public DcMotorEx bLeft;
    public DcMotorEx bRight;

    public Servo ptoServo1;
    public Servo ptoServo2;

    @Override
    public void init(HardwareMap hardwareMap) {
        fLeft = hardwareMap.get(DcMotorEx.class, "fLeft");
        fRight = hardwareMap.get(DcMotorEx.class, "fRight");
        bLeft = hardwareMap.get(DcMotorEx.class, "bLeft");
        bRight = hardwareMap.get(DcMotorEx.class, "bRight");

        fLeft.setDirection(DcMotorEx.Direction.REVERSE);
        fRight.setDirection(DcMotorEx.Direction.FORWARD);
        bLeft.setDirection(DcMotorEx.Direction.REVERSE);
        bRight.setDirection(DcMotorEx.Direction.FORWARD);

        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ptoServo1 = hardwareMap.get(Servo.class, "servo1");
        ptoServo2 = hardwareMap.get(Servo.class, "servo2");

        ptoServo1.setPosition(0.0);
        ptoServo2.setPosition(0.0);

        stopAllMotors();
    }

    @Override
    public void stop() { stopAllMotors(); }

    public void stopAllMotors() {
        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }

    // --------------------------------------
    // Safe slew-rate power set
    // --------------------------------------
    public void setSafePower(DcMotor motor, double targetPower){
        final double SLEW_RATE = 0.2;  // max change per cycle

        double currentPower = motor.getPower();
        double desiredChange = targetPower - currentPower;

        double limitedChange = Math.max(-SLEW_RATE, Math.min(desiredChange, SLEW_RATE));

        motor.setPower(currentPower + limitedChange);
    }

    // --------------------------------------
    // Drive Power
    // --------------------------------------
    public void setDrivePower(double frLeft, double frRight, double baLeft, double baRight) {
        setSafePower(fLeft,  frLeft  * driveSpeedControl);
        setSafePower(fRight, frRight * driveSpeedControl);
        setSafePower(bLeft,  baLeft  * driveSpeedControl);
        setSafePower(bRight, baRight * driveSpeedControl);
    }

    // --------------------------------------
    // TeleOp drive
    // --------------------------------------
    public void driveRobot(Gamepad gamepad1) {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x * 1.1;

        // deadzone
        y  = Math.abs(y)  > 0.10 ? y  : 0;
        x  = Math.abs(x)  > 0.10 ? x  : 0;
        rx = Math.abs(rx) > 0.10 ? rx : 0;

        // stop if no input
        if (y == 0 && x == 0 && rx == 0) {
            stopAllMotors();
            return;
        }

        double frLeft  = y + x + rx;
        double frRight = y - x - rx;
        double baLeft  = y - x + rx;
        double baRight = y + x - rx;

        setDrivePower(frLeft, frRight, baLeft, baRight);
    }
    public void moveForward(double power) {
        setDrivePower(power, power, power, power);
    }
    public void driveForwardTimed(double power, long durationMs) {
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < durationMs) {
            moveForward(power);
            try { Thread.sleep(10); } catch (InterruptedException e) {}
        }
        stopAllMotors();
    }

    //So uhh... Is ts what you mean by servo up and down??
    public void ptoServoUp() {
        ptoServo1.setPosition(1.0); // full up
        ptoServo2.setPosition(1.0);
    }

    // PTO down
    public void ptoServoDown() {
        ptoServo1.setPosition(0.0); // full down
        ptoServo2.setPosition(0.0);
    }

    public void ptoServoSet(double position) {
        ptoServo1.setPosition(position);
        ptoServo2.setPosition(position);
    }
}
