package org.firstinspires.ftc.teamcode.subSystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Lifts implements Subsystems {

    private Servo lLift;
    private Servo rLift;

    public static double LEFT_DOWN  = 0.0;
    public static double LEFT_UP    = 0.4;
    public static double RIGHT_DOWN = 0.4;
    public static double RIGHT_UP   = 0.0;

    private boolean isDown = true;

    @Override
    public void init(HardwareMap hw) {
        lLift = hw.get(Servo.class, "lLift");
        rLift = hw.get(Servo.class, "rLift");
        lLift.setPosition(LEFT_UP);
        rLift.setPosition(RIGHT_UP);
    }

    public void liftsToggle() {
        if (isDown) {
            lLift.setPosition(LEFT_DOWN);
            rLift.setPosition(RIGHT_DOWN);
        } else {
            lLift.setPosition(LEFT_UP);
            rLift.setPosition(RIGHT_UP);
        }
        isDown = !isDown;
    }

    public boolean isDown() {
        return isDown;
    }

    public void stop() {}
}