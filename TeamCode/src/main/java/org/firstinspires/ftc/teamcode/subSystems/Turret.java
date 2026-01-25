package org.firstinspires.ftc.teamcode.subSystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Turret implements Subsystems {

    private DcMotor aimMotor;
    private Limelight3A limelight;

    public static double kp = 0.013;
    public static double ki = 0;
    public static double kd = 0.0004; // derivative term
    public static double kf = 0;

    public static int LEFT_LIMIT = -290;  //730
    public static int RIGHT_LIMIT = 230;//450
    public static int DEFAULT_POSITION = -0; //0

    // For derivative calculation
    private double lastError = 0;

    @Override
    public void init(HardwareMap hardwareMap) {
        aimMotor = hardwareMap.get(DcMotor.class, "aimMotor");
        aimMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        aimMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "Limelight3A");
        limelight.pipelineSwitch(1);
        limelight.setPollRateHz(200);
        limelight.start();
    }

    public void update(double TX) {
        LLResult result = limelight.getLatestResult();
        int currentPos = aimMotor.getCurrentPosition();

        // If no target detected → go to default position
        if (result == null || !result.isValid()) {
            holdDefaultPosition(currentPos);
            lastError = 0; // reset derivative
            return;
        }

        // PID calculation
        double error = TX; // for turret TX-based control
        double derivative = error - lastError;
        lastError = error;

        double motorPower = (kp * error) + (kd * derivative);
        // Clamp movement inside limits
        if (currentPos >= RIGHT_LIMIT && motorPower > 0) motorPower = 0;
        if (currentPos <= LEFT_LIMIT && motorPower < 0) motorPower = 0;

        aimMotor.setPower(motorPower);
    }

    // Moves turret to default position slowly
    private void holdDefaultPosition(int pos) {
        int error = DEFAULT_POSITION - pos;
        double derivative = error - lastError;
        lastError = error;

        double power = (kp * error) + (kd * derivative);

        // *** Slow return-to-default ***
        double MAX_RETURN_SPEED = 0.35;
        if (power > MAX_RETURN_SPEED) power = MAX_RETURN_SPEED;
        if (power < -MAX_RETURN_SPEED) power = -MAX_RETURN_SPEED;

        // Prevent overrunning limits
        if (pos >= RIGHT_LIMIT && power > 0) power = 0;
        if (pos <= LEFT_LIMIT && power < 0) power = 0;

        aimMotor.setPower(power);
    }

    public int getPosition() {
        return aimMotor.getCurrentPosition();
    }

    @Override
    public void stop() {
        aimMotor.setPower(0);
        limelight.stop();
    }

    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    public void addTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        LLResult result = limelight.getLatestResult();
        telemetry.addData("Turret TX", (result != null ? result.getTx() : "null"));
        telemetry.addData("Encoder", aimMotor.getCurrentPosition());
        telemetry.addData("MotorPower", aimMotor.getPower());
        telemetry.addData("Has Target?", result != null && result.isValid());
    }
}