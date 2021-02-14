package org.firstinspires.ftc.teamcode.robot.camera.libs;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.opmodes.auto.Auto;
import org.firstinspires.ftc.teamcode.robot.Controller;
import org.firstinspires.ftc.teamcode.robot.camera.algorithms.RingDetector;
import org.firstinspires.ftc.teamcode.robot.camera.algorithms.WobbleDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static org.firstinspires.ftc.teamcode.robot.camera.libs.OpenCVController.PIPELINE.AUTO;

public class OpenCVController implements Controller{

    private OpenCvCamera openCvPassthrough;
    private Telemetry telemetry;
    private RingDetector ringDetector;
    private WobbleDetector wobbleDetector;

    public enum PIPELINE {
        AUTO, TELE
    }

    public static PIPELINE DEFAULT_PIPELINE = AUTO;

    public OpenCVController(VuforiaLocalizer vuforia, VuforiaLocalizer.Parameters paramaters, int[] viewportContainerIds, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Create a Vuforia passthrough "virtual camera"
        openCvPassthrough = OpenCvCameraFactory.getInstance().createVuforiaPassthrough(vuforia, paramaters, viewportContainerIds[1]);

        openCvPassthrough.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                // Using GPU acceleration can be particularly helpful when using Vuforia passthrough
                // mode, because Vuforia often chooses high resolutions (such as 720p) which can be
                // very CPU-taxing to rotate in software. GPU acceleration has been observed to cause
                // issues on some devices, though, so if you experience issues you may wish to disable it.
                openCvPassthrough.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);

                wobbleDetector = new WobbleDetector(telemetry, false, Auto.getAlliance());
                if (DEFAULT_PIPELINE == AUTO) {
                    ringDetector = new RingDetector(telemetry, false);
                    openCvPassthrough.setPipeline(ringDetector);
                } else {
                    openCvPassthrough.setPipeline(wobbleDetector);
                }

                // We don't get to choose resolution, unfortunately. The width and height parameters
                // are entirely ignored when using Vuforia passthrough mode. However, they are left
                // in the method signature to provide interface compatibility with the other types
                // of cameras.
                openCvPassthrough.startStreaming(0,0, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(openCvPassthrough, 0);
            }
        });
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

    public double getCameraCenterX() {
        return wobbleDetector.getCameraCenterX();
    }

    @Override
    public void init() {

    }

    @Override
    public void start() {
        if (DEFAULT_PIPELINE == AUTO) openCvPassthrough.setPipeline(wobbleDetector);
    }

    @Override
    public void stop() {
//        openCvPassthrough.stopStreaming(); //TODO: this is crashing
        FtcDashboard.getInstance().stopCameraStream();
    }


}
