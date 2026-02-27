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
    private CSensor sensors;
    private DSensor dsensors; //use .getDistanceCM() to get distance in centimeters

    // ===== SLOT MEMORY =====
    private boolean[] slotOccupied = {false,false,false};
    private String[] slotColor = {"None","None","None"};

    // ===== SHOOT STATE =====
    private int shootCase = -1;
    private int shootIndex = 0;
    private boolean rapidMode = false;

    private long shootTimer = 0;

    private static final long ARM_UP = 300;
    private static final long ARM_DOWN = 300;
    private static final long SPIN_TIMEOUT = 1000;

    private boolean lastUp,lastDown,lastLeft,lastRight,lastA,lastB,lastX,lastY;

    @Override
    public void init(){
        drive = new Mecnum(); drive.init(hardwareMap);
        intake = new Intake(); intake.init(hardwareMap);
        spindexer = new Spindexer(); spindexer.init(hardwareMap);
        shooter = new Outtake(); shooter.init(hardwareMap);
        turret = new Turret(); turret.init(hardwareMap);
        sensors = new CSensor(); sensors.init(hardwareMap);
        dsensors = new DSensor(); dsensors.init(hardwareMap);
    }

    @Override
    public void loop(){

        drive.driveRobot(gamepad1);

        handleIntake();
        handleFlywheel();
        handleManualSlotControl();
        handleDpadShoot();

        updateSlotColors();

        runShootStateMachine();
        spindexer.update();
        //==========================================================================
        //Added distance (in milliimeters) to te EEEEEElemetry - thought it might be useful.
        //Remove if not needed.
        telemetry.addData("Distance (mm)", dsensors.getDistanceMM());
        telemetry.update();

        updateLast();
    }

    private void handleIntake(){
        if(gamepad1.a){
            intake.intakeForwards();
        } else if(gamepad1.b){
            intake.intakeReverse();
        } else intake.stop();
    }

    private void handleFlywheel(){
        if(gamepad2.right_trigger > 0.5) shooter.enableFlywheel();
        if(gamepad2.left_trigger > 0.5) shooter.disableFlywheel();
        shooter.updateFlywheel();
    }

    private void handleManualSlotControl(){
        if(shootCase!=-1) return;

        if(gamepad2.a) { spindexer.moveToSlot(1); shootIndex=0; }
        if(gamepad2.b) { spindexer.moveToSlot(2); shootIndex=1; }
        if(gamepad2.x) { spindexer.moveToSlot(3); shootIndex=2; }

        if(gamepad2.y){ // single shot current slot
            rapidMode=false;
            shootCase=0;
        }
    }

    private void handleDpadShoot(){
        if(shootCase!=-1) return;

        // Rapid-fire all 3 slots ignoring color
        if(gamepad2.dpad_up){
            rapidMode=true;
            shootIndex=0;
            shootCase=0;
        }

        // Single-shot any first available
        if(gamepad2.dpad_down){
            rapidMode=false;
            shootIndex=findNextSlot("Any");
            if(shootIndex!=-1) shootCase=0;
        }

        // Shoot first Green
        if(gamepad2.dpad_left){
            rapidMode=false;
            shootIndex=findNextSlot("Green");
            if(shootIndex!=-1) shootCase=0;
        }

        // Shoot first Purple
        if(gamepad2.dpad_right){
            rapidMode=false;
            shootIndex=findNextSlot("Purple");
            if(shootIndex!=-1) shootCase=0;
        }
    }

    private void runShootStateMachine(){
        if(shootCase==-1) return;

        long now = System.currentTimeMillis();

        switch(shootCase){
            case 0: // Move to slot
                spindexer.moveToSlot(shootIndex+1);
                shootTimer=now;
                shootCase=1;
                break;

            case 1: // Wait for spindexer
                if(spindexer.atTarget() || now-shootTimer>SPIN_TIMEOUT){
                    shooter.shooterArmUp();
                    shootTimer=now;
                    shootCase=2;
                }
                break;

            case 2: // Wait for arm up
                if(now-shootTimer>=ARM_UP){
                    shooter.shooterArmDown();
                    shootTimer=now;
                    shootCase=3;
                }
                break;

            case 3: // Finish shot
                if(now-shootTimer>=ARM_DOWN){
                    // Clear slot memory
                    slotOccupied[shootIndex]=false;
                    slotColor[shootIndex]="None";

                    if(rapidMode){
                        shootIndex++;
                        if(shootIndex<3){
                            shootCase=0; // fire next slot
                        } else {
                            shootCase=-1; // done
                            rapidMode=false;
                        }
                    } else {
                        shootCase=-1; // single shot done
                    }
                }
                break;
        }
    }

    // Update slot colors continuously from sensors
    private void updateSlotColors(){
        for(int i=0;i<3;i++){
            if(sensors.ballPresent(i)){
                slotOccupied[i]=true;
                slotColor[i]=sensors.getColor(i);
            } else {
                slotOccupied[i]=false;
                slotColor[i]="None";
            }
        }
    }

    private int findNextSlot(String color){
        for(int i=0;i<3;i++){
            if(slotOccupied[i] && (color.equals("Any") || slotColor[i].equals(color))){
                return i;
            }
        }
        return -1;
    }

    private void updateLast(){
        lastUp=gamepad2.dpad_up;
        lastDown=gamepad2.dpad_down;
        lastLeft=gamepad2.dpad_left;
        lastRight=gamepad2.dpad_right;
        lastA=gamepad2.a;
        lastB=gamepad2.b;
        lastX=gamepad2.x;
        lastY=gamepad2.y;
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