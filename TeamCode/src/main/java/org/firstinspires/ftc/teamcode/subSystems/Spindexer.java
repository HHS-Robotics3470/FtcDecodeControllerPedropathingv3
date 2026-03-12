package org.firstinspires.ftc.teamcode.subSystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Spindexer implements Subsystems {

    private DcMotor motor;

    public static double kP = 0.01;
    public static double kI = 0.00001;
    public static double kD = 0.0008;

    public static double MAX_POWER = 0.7;

    private static final int TICKS_PER_REV = 384;
    private static final int SLOT_ANGLE = TICKS_PER_REV / 3;

    private int targetPos = 0;

    private double integral = 0;
    private double lastError = 0;
    private long lastTime = 0;

    private boolean[] slotFilled = {false,false,false};

    @Override
    public void init(HardwareMap hw) {

        motor = hw.get(DcMotor.class,"spindexerMotor");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lastTime = System.currentTimeMillis();
    }

    public void moveToSlot(int slot){

        if(slot < 1 || slot > 3) return;

        int desired = (slot-1)*SLOT_ANGLE;

        int currentMod = mod(motor.getCurrentPosition(),TICKS_PER_REV);

        int diff = desired - currentMod;

        if(diff > TICKS_PER_REV/2) diff -= TICKS_PER_REV;
        if(diff < -TICKS_PER_REV/2) diff += TICKS_PER_REV;

        targetPos = motor.getCurrentPosition() + diff;

        integral = 0;
        lastError = 0;
    }

    public int getNextOpenSlot(){

        for(int i=0;i<3;i++){

            if(!slotFilled[i]){
                return i+1;
            }
        }

        return -1;
    }

    public void markSlotFilled(int slot){

        if(slot>=1 && slot<=3){
            slotFilled[slot-1] = true;
        }
    }

    public void clearSlot(int slot){

        if(slot>=1 && slot<=3){
            slotFilled[slot-1] = false;
        }
    }

    public void update(){

        long now = System.currentTimeMillis();
        double dt = (now-lastTime)/1000.0;

        lastTime = now;

        if(dt<=0) dt = 0.01;

        int current = motor.getCurrentPosition();

        double error = targetPos-current;

        integral += error*dt;

        double derivative = (error-lastError)/dt;

        lastError = error;

        double output =
                (kP*error)+
                        (kI*integral)+
                        (kD*derivative);

        output = clamp(output,-MAX_POWER,MAX_POWER);

        motor.setPower(output);
    }

    public boolean atTarget(){

        return Math.abs(targetPos - motor.getCurrentPosition()) < 6;
    }

    @Override
    public void stop(){

        motor.setPower(0);
    }

    private int mod(int x,int m){

        return (x % m + m) % m;
    }

    private double clamp(double val,double min,double max){

        return Math.max(min,Math.min(max,val));
    }
}