package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CSensor implements Subsystems{

    private RevColorSensorV3 sensor;

    @Override
    public void init(HardwareMap hardwareMap) {
        sensor = hardwareMap.get(RevColorSensorV3.class, "light_yagami");
        sensor.enableLed(true);

    }

    @Override
    public void stop() {

    }
}