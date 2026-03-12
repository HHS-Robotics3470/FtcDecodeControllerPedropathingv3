package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subSystems.*;

@Configurable
@TeleOp(name="TeleOpBlue", group="OpMode")
public class TeleOPBlue extends OpMode {

    private Mecnum drive;
    private Intake intake;
    private Spindexer spindexer;
    private Outtake shooter;
    private Turret turret;
    private DSensor dsensors;

    private boolean ballDetected=false;
    private long detectTime=0;
    private boolean waiting=false;

    private int currentSlot=0;

    @Override
    public void init(){

        drive = new Mecnum(); drive.init(hardwareMap);
        intake = new Intake(); intake.init(hardwareMap);
        spindexer = new Spindexer(); spindexer.init(hardwareMap);
        shooter = new Outtake(); shooter.init(hardwareMap);
        turret = new Turret(); turret.init(hardwareMap);
        dsensors = new DSensor(); dsensors.init(hardwareMap);
    }

    @Override
    public void loop(){

        drive.driveRobot(gamepad1);

        handleIntake();
        handleFlywheel();
        handleManualSlots();

        autoIndex();

        spindexer.update();

        if(shooter.wasShotTriggered()){

            spindexer.clearSlot(currentSlot);
        }

        telemetry.addData("Distance",dsensors.getDistanceMM());
        telemetry.update();
    }

    private void handleIntake(){

        if(gamepad1.a){

            intake.intakeForwards();
        }

        else if(gamepad1.b){

            intake.intakeReverse();
        }

        else{

            intake.stop();
        }
    }

    private void handleFlywheel(){

        if(gamepad2.right_trigger>0.5)
            shooter.enableFlywheel();

        if(gamepad2.left_trigger>0.5)
            shooter.disableFlywheel();

        shooter.updateFlywheel();
    }

    private void handleManualSlots(){

        if(gamepad2.a){

            spindexer.moveToSlot(1);
            currentSlot=1;
        }

        if(gamepad2.b){

            spindexer.moveToSlot(2);
            currentSlot=2;
        }

        if(gamepad2.x){

            spindexer.moveToSlot(3);
            currentSlot=3;
        }
    }

    private void autoIndex(){
        double distance = dsensors.getDistanceMM();
        if(gamepad1.a && distance<200 && !ballDetected){
            ballDetected=true;
            detectTime = System.currentTimeMillis();
            waiting=true;
        }
        if(waiting){
            if(System.currentTimeMillis()-detectTime>1000){
                int slot = spindexer.getNextOpenSlot();
                if(slot!=-1){
                    spindexer.moveToSlot(slot);
                    spindexer.markSlotFilled(slot);
                    currentSlot = slot;
                }
                waiting=false;
            }
        }
        if(distance>220){
            ballDetected=false;
        }
    }

    @Override
    public void stop(){

        drive.stop();
        intake.stop();
        spindexer.stop();
        shooter.stop();
        turret.stop();
    }
}