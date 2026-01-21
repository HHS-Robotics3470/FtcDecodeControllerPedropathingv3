package org.firstinspires.ftc.teamcode.subSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Spindexer implements Subsystems {

    private Servo spindexerServo;
    public RevBlinkinLedDriver underGlow;
    //public RevBlinkinLedDriver rightLights;

    public static double HOLD_1 = 0.41;
    public static double HOLD_2 = 0.038;
    public static double HOLD_3 = 0.8;

    public static double OUTTAKE_1 = 1;
    public static double OUTTAKE_2 = 0.6;
    public static double OUTTAKE_3 = 0.23;

    private double currentPosition = HOLD_1;

    @Override
    public void init(HardwareMap hardwareMap) {
        spindexerServo = hardwareMap.get(Servo.class, "spindexerServo");
        underGlow = hardwareMap.get(RevBlinkinLedDriver.class,"underGlow");
        spindexerServo.setPosition(HOLD_1);
        currentPosition = HOLD_1;
    }

    public void moveToHold(int slot) {
        switch (slot) {
            case 1: spindexerServo.setPosition(HOLD_1); currentPosition = HOLD_1; break;
            case 2: spindexerServo.setPosition(HOLD_2); currentPosition = HOLD_2; break;
            case 3: spindexerServo.setPosition(HOLD_3); currentPosition = HOLD_3; break;
        }
    }

    public void moveHoldToOuttake(int slot) {
        switch (slot) {
            case 1: spindexerServo.setPosition(OUTTAKE_1); currentPosition = OUTTAKE_1; break;
            case 2: spindexerServo.setPosition(OUTTAKE_2); currentPosition = OUTTAKE_2; break;
            case 3: spindexerServo.setPosition(OUTTAKE_3); currentPosition = OUTTAKE_3; break;
        }
    }

    public boolean isInOuttake() {
        return currentPosition == OUTTAKE_1
                || currentPosition == OUTTAKE_2
                || currentPosition == OUTTAKE_3;
    }

    public double getCurrentPosition() {
        return currentPosition;
    }

    public void blinkinRed() {
        underGlow.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
    }
    public void blinkinGreen() {
        underGlow.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }
    public void blinkinBlue() {
        underGlow.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
    }

    public void blinkinOrange() {
        underGlow.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
    }
    public void blinkinBlack() {
        underGlow.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    public void timeTelemetry() {
        telemetry.addData("Timer", underGlow);
        telemetry.update();
    }
    @Override
    public void stop() {
        spindexerServo.setPosition(currentPosition);
    }
}