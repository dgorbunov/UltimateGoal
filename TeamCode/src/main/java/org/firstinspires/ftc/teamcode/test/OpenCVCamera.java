package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.Controller;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;


public class OpenCVCamera extends OpenCvPipeline implements Controller {
    private OpenCvCamera openCvCamera; //Camera Object\
    private HardwareMap hardwareMap;

    public OpenCVCamera(OpenCvCamera camera, int width, int height, OpenCvCameraRotation cameraRotation, HardwareMap hardwareMap) {
        openCvCamera = camera;
        openCvCamera.openCameraDevice();
        openCvCamera.setPipeline(this);
        openCvCamera.startStreaming(width, height, cameraRotation);
        this.hardwareMap = hardwareMap;

    }

    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    }



    public void start() { }
    public void stop() { }

    @Override
    public Mat processFrame(Mat input) {
        Mat inputrgb = new Mat();
        Imgproc.cvtColor(input, inputrgb, Imgproc.COLOR_RGBA2RGB);
        return input;
    }
}
