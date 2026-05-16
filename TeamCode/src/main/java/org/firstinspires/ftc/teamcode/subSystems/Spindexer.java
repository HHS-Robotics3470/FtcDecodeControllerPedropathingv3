package org.firstinspires.ftc.teamcode.subSystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Spindexer implements Subsystems {

    private DcMotorEx motor;


    public static double MOTOR_POWER = 0.16;
    public static int POSITION_THRESHOLD = 30;

    private static final double TICKS_PER_REV = 537.6;

    private int targetPos = 0;
    private boolean isMoving = false;

    private boolean[] slotFilled = {false, false, false};

    @Override
    public void init(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "spindexerMotor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        targetPos = motor.getCurrentPosition();
    }

    public void moveToSlot(int slot) {
        if (slot < 1 || slot > 3) return;

        int currentPos = motor.getCurrentPosition();
        int desired = (int)((slot - 1) * TICKS_PER_REV / 3);

        int currentMod = mod(currentPos, (int)TICKS_PER_REV);

        int diff = desired - currentMod;

        if (diff > TICKS_PER_REV / 2) diff -= (int)TICKS_PER_REV;
        if (diff < -TICKS_PER_REV / 2) diff += (int)TICKS_PER_REV;

        if (Math.abs(diff) <= POSITION_THRESHOLD) return;

        targetPos = currentPos + diff;

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(diff > 0 ? MOTOR_POWER : -MOTOR_POWER);

        isMoving = true;
    }

    public void update() {
        if (!isMoving) return;

        if (Math.abs(targetPos - motor.getCurrentPosition()) <= POSITION_THRESHOLD) {
            motor.setPower(0);
            isMoving = false;
        }
    }

    public boolean atTarget() {
        return !isMoving;
    }

    public int getNextOpenSlot() {
        for (int i = 0; i < 3; i++) {
            if (!slotFilled[i]) return i + 1;
        }
        return -1;
    }

    public void markSlotFilled(int slot) {
        if (slot >= 1 && slot <= 3) slotFilled[slot - 1] = true;
    }

    public void clearSlot(int slot) {
        if (slot >= 1 && slot <= 3) slotFilled[slot - 1] = false;
    }

    public int getPosition() { return motor.getCurrentPosition(); }
    public int getTarget() { return targetPos; }
    public boolean getIsMoving() { return isMoving; }

    @Override
    public void stop() {
        motor.setPower(0);
        isMoving = false;
    }

    private int mod(int x, int m) {
        return (x % m + m) % m;
    }
}