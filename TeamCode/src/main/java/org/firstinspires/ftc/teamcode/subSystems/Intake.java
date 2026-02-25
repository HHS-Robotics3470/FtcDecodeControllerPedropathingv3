package org.firstinspires.ftc.teamcode.subSystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Intake implements Subsystems {

    private DcMotor intakeMotor;
    private Servo intakeServo;

    public static double INTAKE_POWER = 1.0;
    public static double OUTTAKE_POWER = -1.0;
    public static double INTAKE_SERVO_IN = 0.75;
    public static double INTAKE_SERVO_OUT = 1.0;

    @Override
    public void init(HardwareMap hardwareMap){
        intakeMotor = hardwareMap.get(DcMotor.class,"intakeMotor");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(0);
        intakeServo = hardwareMap.get(Servo.class,"intakeServo");
        intakeServo.setPosition(INTAKE_SERVO_IN);
    }

    @Override
    public void stop(){
        intakeMotor.setPower(0);
        intakeServo.setPosition(INTAKE_SERVO_IN);
    }

    public void intakeForwards(){
        intakeMotor.setPower(INTAKE_POWER);
        intakeServo.setPosition(INTAKE_SERVO_OUT);
    }

    public void intakeReverse(){
        intakeMotor.setPower(OUTTAKE_POWER);
        intakeServo.setPosition(INTAKE_SERVO_OUT);
    }

    public void setServoIn(){ intakeServo.setPosition(INTAKE_SERVO_IN); }

    // =============================
    // NEW BALL CHECK FOR SLOT 1
    // =============================
}