package org.firstinspires.ftc.teamcode.subSystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Outtake implements Subsystems {

    private DcMotorEx flywheelMotor1;
    private Servo hoodServo;
    private Servo shootServo;

    public static double HOOD_MIN = 0.0;
    public static double HOOD_MAX = 0.55;

    public static double SHOOTER_ARM_DOWN = 0.97;
    public static double SHOOTER_ARM_UP = 0.6;

    public static double MOTOR_POWER = 1.0;

    private boolean flywheelOn = false;
    private boolean armDown = true;

    private boolean shotTriggered = false;

    @Override
    public void init(HardwareMap hw){

        flywheelMotor1 = hw.get(DcMotorEx.class,"flywheelMotor1");
        flywheelMotor1.setDirection(DcMotorSimple.Direction.REVERSE);

        hoodServo = hw.get(Servo.class,"hoodServo");
        shootServo = hw.get(Servo.class,"shootServo");

        hoodServo.setPosition(HOOD_MIN);
        shootServo.setPosition(SHOOTER_ARM_DOWN);
    }

    public void shooterArmUp(){

        shootServo.setPosition(SHOOTER_ARM_UP);

        armDown = false;

        shotTriggered = true;
    }

    public void shooterArmDown(){

        shootServo.setPosition(SHOOTER_ARM_DOWN);

        armDown = true;
    }

    public boolean wasShotTriggered(){

        if(shotTriggered){

            shotTriggered = false;

            return true;
        }

        return false;
    }

    public void enableFlywheel(){

        flywheelOn = true;
    }

    public void disableFlywheel(){

        flywheelOn = false;

        flywheelMotor1.setPower(0);
    }

    public void updateFlywheel(){

        if(!flywheelOn){

            flywheelMotor1.setPower(0);

            return;
        }

        flywheelMotor1.setPower(MOTOR_POWER);
    }

    @Override
    public void stop(){

        disableFlywheel();

        hoodServo.setPosition(HOOD_MIN);

        shootServo.setPosition(SHOOTER_ARM_DOWN);
    }
}