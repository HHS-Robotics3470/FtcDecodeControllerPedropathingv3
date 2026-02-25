package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CSensor implements Subsystems {

    private RevColorSensorV3[] sensors = new RevColorSensorV3[3];

    @Override
    public void init(HardwareMap hw){
        sensors[0] = hw.get(RevColorSensorV3.class,"colorSensor1");
        sensors[1] = hw.get(RevColorSensorV3.class,"colorSensor2");
        sensors[2] = hw.get(RevColorSensorV3.class,"colorSensor3");

        for(RevColorSensorV3 s : sensors){
            s.enableLed(true);
        }
    }

    public boolean ballPresent(int i){
        String color = detectColor(i);
        return color.equals("Green") || color.equals("Purple");
    }

    public String getColor(int i){
        return detectColor(i);
    }

    // Super-sensitive detectColor
    private String detectColor(int i){
        int r = sensors[i].red();
        int g = sensors[i].green();
        int b = sensors[i].blue();
        int total = r + g + b;

        if(total < 200) return "None";  // very sensitive

        double rRatio = (double) r / total;
        double gRatio = (double) g / total;
        double bRatio = (double) b / total;

        // GREEN detection
        if(gRatio > 0.25 && g > 30) return "Green";

        // PURPLE detection
        if(bRatio > 0.25 && gRatio < 0.45) return "Purple";

        return "None";
    }

    public String getRawRGB(int i){
        return sensors[i].red() + ", " +
                sensors[i].green() + ", " +
                sensors[i].blue();
    }

    @Override
    public void stop(){}
}