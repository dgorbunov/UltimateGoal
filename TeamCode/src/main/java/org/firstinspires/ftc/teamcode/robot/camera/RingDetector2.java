package org.firstinspires.ftc.teamcode.robot.camera;

import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class RingDetector2 extends OpenCvPipeline {

    OpenCvCamera camera;
    Telemetry telemetry;

    public RingDetector2(OpenCvCamera camera, Telemetry telemetry) {
        this.camera = camera;
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        return null;
    }


}
