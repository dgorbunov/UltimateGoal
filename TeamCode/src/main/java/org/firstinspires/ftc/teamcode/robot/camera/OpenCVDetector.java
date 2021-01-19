package org.firstinspires.ftc.teamcode.robot.camera;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.robot.Controller;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class OpenCVDetector extends OpenCvPipeline implements Controller{

    OpenCvCamera openCvPassthrough;
    Telemetry telemetry;

    public OpenCVDetector(VuforiaLocalizer vuforia, VuforiaLocalizer.Parameters paramaters, int[] viewportContainerIds, Telemetry telemetry ) {
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
                openCvPassthrough.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
                openCvPassthrough.setPipeline(new OpenCVDetector());

                // We don't get to choose resolution, unfortunately. The width and height parameters
                // are entirely ignored when using Vuforia passthrough mode. However, they are left
                // in the method signature to provide interface compatibility with the other types
                // of cameras.
                openCvPassthrough.startStreaming(0,0, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(openCvPassthrough, 0);
            }
        });
    }

    private OpenCVDetector() {

    }

    public int getRingCount() {
        return 0;
        //TODO: Implement
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.rectangle(
                input,
                new Point(
                        input.cols()/4,
                        input.rows()/4),
                new Point(
                        input.cols()*(3f/4f),
                        input.rows()*(3f/4f)),
                new Scalar(255,0,0,255), 4);

        return input;
    }

    public void printStats() {
        telemetry.addData("Passthrough FPS", openCvPassthrough.getFps());
        telemetry.addData("Frame count", openCvPassthrough.getFrameCount());
        telemetry.update();
    }

    @Override
    public void init() {

    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
//        openCvPassthrough.stopStreaming(); //TODO: this? or stop camera async
        FtcDashboard.getInstance().stopCameraStream();
    }


}
