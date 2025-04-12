package opmode.blue;

import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LLTestingWithMath extends opmode.blue.LLResultStuff {
    private LLResult result;
    double degreesTy;
    double targetHeight = 1.9;
    double cameraHeight = 17.7;

    double cameraAngle = Math.toRadians(0);

    public LLTestingWithMath() {
        degreesTy = result.getTy();
    }

    public double distanceSampleLL = (targetHeight - cameraHeight) / Math.tan(cameraAngle + degreesTy);

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        super.telemetry.addData("distance from limelight to sample" ,distanceSampleLL);
    }
}

