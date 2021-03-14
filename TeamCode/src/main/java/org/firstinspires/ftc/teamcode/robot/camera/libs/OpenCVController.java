package org.firstinspires.ftc.teamcode.robot.camera.libs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.opmodes.auto.Auto;
import org.firstinspires.ftc.teamcode.robot.Controller;
import org.firstinspires.ftc.teamcode.robot.camera.CameraController;
import org.firstinspires.ftc.teamcode.robot.camera.algorithms.RingDetector;
import org.firstinspires.ftc.teamcode.robot.camera.algorithms.VerticalRingDetector;
import org.firstinspires.ftc.teamcode.robot.camera.algorithms.WobbleDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static org.firstinspires.ftc.teamcode.robot.camera.libs.OpenCVController.PIPELINE.AUTO;

public class OpenCVController implements Controller{

    private OpenCvCamera openCvCamera;
    private Telemetry telemetry;
    private RingDetector ringDetector;
    private WobbleDetector wobbleDetector;
    private VerticalRingDetector verticalDetector;

    public enum PIPELINE {
        AUTO, TELE
    }

    public static PIPELINE DEFAULT_PIPELINE = AUTO;

    public OpenCVController(VuforiaLocalizer vuforia, VuforiaLocalizer.Parameters paramaters, int[] viewportContainerIds, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Creates a Vuforia passthrough "virtual camera"
        openCvCamera = OpenCvCameraFactory.getInstance().createVuforiaPassthrough(vuforia, paramaters, viewportContainerIds[1]);

        openCvCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Using GPU acceleration can be particularly helpful when using Vuforia passthrough
                // mode, because Vuforia often chooses high resolutions (such as 720p) which can be
                // very CPU-taxing to rotate in software. GPU acceleration has been observed to cause
                // issues on some devices, though, so if you experience issues you may wish to disable it.
//                openCvCamera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
                //TODO: Evaluate if this is the cause of our crashes
                setPipeline();

                // We don't get to choose resolution, unfortunately. The width and height parameters
                // are entirely ignored when using Vuforia passthrough mode. However, they are left
                // in the method signature to provide interface compatibility with the other types
                // of cameras.
                openCvCamera.startStreaming(0, 0, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(openCvCamera, 0);
            }
        });
    }

    public OpenCVController(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Creates an exclusive OpenCv camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        openCvCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, CameraController.WEBCAM_NAME), cameraMonitorViewId);

        openCvCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                setPipeline();
                openCvCamera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(openCvCamera, 0);
            }
        });
    }

    private void setPipeline() {
        if (openCvCamera != null) {
            if (DEFAULT_PIPELINE == AUTO) {
                ringDetector = new RingDetector(telemetry, false);
                wobbleDetector = new WobbleDetector(telemetry, false, Auto.getAlliance());
                openCvCamera.setPipeline(ringDetector);
            } else {
                verticalDetector = new VerticalRingDetector(telemetry, false);
                openCvCamera.setPipeline(verticalDetector);
            }
        }
    }

    public int getRingCount() {
        if (ringDetector != null) return ringDetector.getRingCount();
        else return -1;
    }

    public String getRingCountStr() {
        if (ringDetector != null) return ringDetector.getRingCountStr();
        else return null;
    }

    public double getWobbleDisplacement() {
       return wobbleDetector.getDisplacementFromCenter();
    }

    public double getVerticalRingDisplacement() { return verticalDetector.getDisplacementFromCenter(); }

    @Override
    public void init() {

    }

    @Override
    public void start() { }

    public void closeCamera() {
        openCvCamera.closeCameraDevice();
    }

    @Override
    public void stop() {
//        openCvCamera.stopStreaming(); //TODO: this is crashing
        FtcDashboard.getInstance().stopCameraStream();
    }


}
