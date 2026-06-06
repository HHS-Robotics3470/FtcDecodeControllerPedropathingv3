package org.firstinspires.ftc.teamcode.subSystems;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@Configurable
public class Lifts implements Subsystems {
    private Servo lLift;
    private Servo rLift;
    public static double LEFT_DOWN = 0.3; //place holder
    public static double LEFT_UP = 0.3; //place holder
    public static double RIGHT_DOWN = 0.7; //mirrored place holder
    public static double RIGHT_UP = 0.3; //mirrored place holder
    private boolean isDown = true;

    @Override
    public void init(HardwareMap hw){
        lLift = hw.get(Servo.class, "lLift");
        rLift = hw.get(Servo.class, "rLift");
        lLift.setPosition(LEFT_DOWN);
        rLift.setPosition(RIGHT_DOWN);
    }
    public void liftsToggle(){
        if (isDown){
            lLift.setPosition(LEFT_UP);
            rLift.setPosition(RIGHT_UP);
        }
        else{
            lLift.setPosition(LEFT_DOWN);
            rLift.setPosition(RIGHT_DOWN);
        }
    }
    public boolean isDown(){
        return isDown;
    }
    public void stop(){}
}