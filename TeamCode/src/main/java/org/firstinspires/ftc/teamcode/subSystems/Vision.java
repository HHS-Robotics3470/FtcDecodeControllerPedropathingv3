
package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class Vision implements Subsystems {

    private Limelight3A camera;

    private double tx = 0;
    private double ty = 0;
    private double ta = 0;

    private int targetId = -1;

    @Override
    public void init(HardwareMap hardwareMap) {
        camera = hardwareMap.get(Limelight3A.class, "Limelight3A");
        camera.start();
    }

    public void setTargetId(int id) {
        targetId = id;
    }

    public void update() {
        LLResult result = camera.getLatestResult();
        tx = 0; ty = 0; ta = 0;

        if (result == null || !result.isValid()) return;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult f : fiducials) {
            if (f.getFiducialId() == targetId) {
                tx = f.getTargetXDegrees();
                ty = f.getTargetYDegrees();
                ta = f.getTargetArea();
                return;
            }
        }
    }

    public double getTX() { return tx; }
    public double getTY() { return ty; }
    public double getDistanceArea() { return ta; }

    // Angle-based distance estimate (fallback if pose unavailable)
    public double getDistanceTrig() {
        double cameraHeight = 11.0; // inches
        double tagHeight = 29.0;    // inches
        double cameraAngle = 20.0;  // degrees
        double totalAngle = Math.toRadians(cameraAngle + ty);
        return (tagHeight - cameraHeight) / Math.tan(totalAngle);
    }

    @Override
    public void stop() {
        camera.stop();
    }
}
