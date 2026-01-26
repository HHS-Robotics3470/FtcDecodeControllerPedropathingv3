package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class Vision implements Subsystems {

    private Limelight3A camera;
    private GoBildaPinpointDriver pinpoint;

    private double tx;
    private double ta;
    private double distance;
    private double heading;
    private Pose3D botpose;
    private int targetId = -1;

    @Override
    public void init(HardwareMap hardwareMap) {
        camera = hardwareMap.get(Limelight3A.class, "Limelight3A");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        camera.pipelineSwitch(1);
        camera.start();
    }

    public void setTargetId(int id) {
        targetId = id;
    }

    @Override
    public void stop() {
        camera.stop();
    }

    public void updateVision() {
        heading = pinpoint.getHeading(AngleUnit.DEGREES);
        camera.updateRobotOrientation(heading);

        tx = 0;
        ta = 0;
        distance = 0;
        botpose = null;

        LLResult result = camera.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null) {
                for (LLResultTypes.FiducialResult f : fiducials) {
                    if (f.getFiducialId() == targetId) {
                        tx = f.getTargetXDegrees();
                        ta = f.getTargetArea();
                        distance = calculateDistance(ta);
                        botpose = result.getBotpose_MT2();
                        break;
                    }
                }
            }
        }
    }

    // ===== TA → DISTANCE TABLE =====
    private final double[] taValues = {
            0.0317, 0.0258, 0.023, 0.0185, 0.0162, 0.0124, 0.0104, 0.0086,
            0.0076, 0.0068, 0.005, 0.0045, 0.0041, 0.0036, 0.0032, 0.0027, 0.0024
    };

    private final double[] distances = {
            36, 40, 44, 48, 52, 60, 66, 72,
            78, 82, 96, 102, 108, 120, 126, 132, 141
    };

    private double calculateDistance(double ta) {
        if (ta <= 0) return 0;

        for (int i = 0; i < taValues.length - 1; i++) {
            if (ta <= taValues[i] && ta >= taValues[i + 1]) {
                double ratio =
                        (taValues[i] - ta) /
                                (taValues[i] - taValues[i + 1]);

                return distances[i] +
                        ratio * (distances[i + 1] - distances[i]);
            }
        }

        if (ta > taValues[0]) return distances[0];
        return distances[distances.length - 1];
    }

    public double getDistance() { return distance; }
    public double getTX() { return tx; }
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Distance (in)", distance);
        telemetry.addData("TX", tx);
        telemetry.addData("TA", ta);
    }
}
