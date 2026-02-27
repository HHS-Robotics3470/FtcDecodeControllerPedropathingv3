package org.firstinspires.ftc.teamcode.subSystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Spindexer implements Subsystems {

    private DcMotor motor;

    // ====== TUNABLE PID ======
    public static double kP = 0.01;
    public static double kI = 0.00001;
    public static double kD = 0.0008;
    public static double kF = 0.0000001;        // optional feedforward

    public static double MAX_POWER = 0.7;
    public static double INTEGRAL_LIMIT = 300;

    private static final int TICKS_PER_REV = 384;
    private static final int SLOT_ANGLE = TICKS_PER_REV / 3;

    private int targetPos = 0;

    private double integral = 0;
    private double lastError = 0;
    private long lastTime = 0;

    @Override
    public void init(HardwareMap hw) {
        motor = hw.get(DcMotor.class, "spindexerMotor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lastTime = System.currentTimeMillis();
    }

    // ==========================================
    // Move to slot (Shortest rotational path)
    // ==========================================
    public void moveToSlot(int slot) {
        if (slot < 1 || slot > 3) return;

        int desired = (slot - 1) * SLOT_ANGLE;

        int currentMod = mod(motor.getCurrentPosition(), TICKS_PER_REV);
        int diff = desired - currentMod;

        if (diff > TICKS_PER_REV / 2) diff -= TICKS_PER_REV;
        if (diff < -TICKS_PER_REV / 2) diff += TICKS_PER_REV;

        targetPos = motor.getCurrentPosition() + diff;

        // Reset PID accumulation for clean move
        integral = 0;
        lastError = 0;
    }

    // ==========================================
    // PID UPDATE LOOP
    // ==========================================
    public void update() {

        long now = System.currentTimeMillis();
        double dt = (now - lastTime) / 1000.0;
        lastTime = now;

        if (dt <= 0) dt = 0.01;

        int current = motor.getCurrentPosition();
        double error = targetPos - current;

        // Integral with clamp (anti-windup)
        integral += error * dt;
        integral = clamp(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

        double derivative = (error - lastError) / dt;
        lastError = error;

        double output =
                (kP * error) +
                        (kI * integral) +
                        (kD * derivative) +
                        (Math.signum(error) * kF);

        output = clamp(output, -MAX_POWER, MAX_POWER);

        motor.setPower(output);
    }

    public boolean atTarget() {
        return Math.abs(targetPos - motor.getCurrentPosition()) < 6;
    }

    @Override
    public void stop() {
        motor.setPower(0);
    }

    // ===============================
    // Helpers
    // ===============================
    private int mod(int x, int m) {
        return (x % m + m) % m;
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}