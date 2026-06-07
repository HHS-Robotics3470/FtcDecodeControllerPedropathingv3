package org.firstinspires.ftc.teamcode.subSystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Spindexer implements Subsystems {

    private DcMotorEx motor;

    public static double MOVE_POWER = 0.1;
    public static double BRAKE_POWER = 0.5; // prevents overshooting
    public static long   MOVE_TIME_MS = 150;
    public static long   BRAKE_TIME_MS = 50;

    // state machine
    private static final int IDLE = 0;
    private static final int MOVING = 1;
    private static final int BRAKING = 2;

    private int state = IDLE;
    private long stateTimer = 0;
    private double moveDirection = 0;

    private int currentSlot = 1;
    private int targetSlot = 1;

    private boolean[] slotFilled = {false, false, false};

    @Override
    public void init(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "spindexerMotor");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);
        // once init: current pos is set to pos 1
        currentSlot = 1;
        targetSlot  = 1;
    }

    public void moveToSlot(int slot) {
        if (slot < 1 || slot > 3) return;
        if (slot == currentSlot && state == IDLE) return;
        if (slot == targetSlot && state != IDLE) return;

        targetSlot = slot;

        //***** Spindexing Logic *****
        // forward sequence 1->2, 2->3, 3->1 = clockwise = negative
        // backward sequence 1->3, 3->2, 2->1 = counter-clockwise = positive
        boolean isForward = (currentSlot == 1 && targetSlot == 2)
                || (currentSlot == 2 && targetSlot == 3)
                || (currentSlot == 3 && targetSlot == 1);

        moveDirection = isForward ? -1.0 : 1.0;

        motor.setPower(MOVE_POWER * moveDirection);
        stateTimer = System.currentTimeMillis();
        state = MOVING;
    }

    public void update() {
        long now = System.currentTimeMillis();

        switch (state) {
            case MOVING:
                if (now - stateTimer >= MOVE_TIME_MS) {
                    motor.setPower(-BRAKE_POWER * moveDirection);
                    stateTimer = now;
                    state = BRAKING;
                }
                break;

            case BRAKING:
                if (now - stateTimer >= BRAKE_TIME_MS) {
                    motor.setPower(0);
                    currentSlot = targetSlot;
                    state = IDLE;
                }
                break;

            case IDLE:
            default:
                break;
        }
    }

    public boolean atTarget() {
        return state == IDLE;
    }

    public boolean getActive() {
        return state != IDLE;
    }

    public int getCurrentSlot() {
        return currentSlot;
    }

    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public int getTargetPosition() {
        return targetSlot;
    }

    public String getMode() {
        return motor.getMode().toString();
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

    @Override
    public void stop() {
        motor.setPower(0);
        state = IDLE;
    }

    private int mod(int x, int m) {
        return (x % m + m) % m;
    }
}