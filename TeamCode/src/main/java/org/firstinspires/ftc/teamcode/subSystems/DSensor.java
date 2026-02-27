package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DSensor implements Subsystems {

    private DistanceSensor dsensor;

    @Override
    public void init(HardwareMap hw) {
        dsensor = hw.get(DistanceSensor.class, "distanceSensor");
    }

    public double getDistanceMM() {
        return dsensor.getDistance(DistanceUnit.MM);
    }

    @Override
    public void stop() {}
}