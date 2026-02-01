package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CSensor implements Subsystems {

    private RevColorSensorV3 sensor;

    @Override
    public void init(HardwareMap hardwareMap) {
        sensor = hardwareMap.get(RevColorSensorV3.class, "light_yagami");
        sensor.enableLed(true);
    }

    // Returns "Purple", "Green", or "Other" based on RGB readings
    public String getColor() {
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();

        // Purple: 238,166,222
        if (r > 200 && g > 150 && g < 180 && b > 200) {
            return "Purple";
        }
        // Green: 0,76,51
        else if (g > 50 && g < 100 && r < 50 && b < 60) {
            return "Green";
        }
        else {
            return "Other";
        }
    }
    public int getRed() { return sensor.red(); }
    public int getGreen() { return sensor.green(); }
    public int getBlue() { return sensor.blue(); }


    @Override
    public void stop() {
        // Nothing needed here
    }
}
