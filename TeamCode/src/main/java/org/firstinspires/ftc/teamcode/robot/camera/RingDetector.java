package org.firstinspires.ftc.teamcode.robot.camera;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@Deprecated
public class RingDetector extends OpenCvPipeline {

    //TODO: Make vision alogirithms into seperate classes

    OpenCvCamera webcam;
    Telemetry telemetry;

    boolean viewportPaused;
    static int ringCount;
    final int[] Rect1Pos = {0,0};
    final int[] Rect2Pos = {0,0};
    final int SingleRingHeight = 20;
    final int QuadRingHeight = 69;

    /*
    OpenCV Coordinate System
    ______________________
    |*(0,0)
    |
    |
    |          (640, 480)*
     */

    Mat YCbCr = new Mat();
    Mat output = new Mat();
    Mat upperCrop = new Mat();
    Mat lowerCrop = new Mat();

    public RingDetector(OpenCvCamera webcam, Telemetry telemetry) {
        this.webcam = webcam;
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {

        input.copyTo(output); //copy input over

        Imgproc.cvtColor(output, YCbCr, Imgproc.COLOR_RGB2YCrCb); //convert to YCbCr color space

        //Make our rectangles
        Rect rect1 = new Rect(Rect1Pos[0], Rect1Pos[1], 119, QuadRingHeight);
        Rect rect2 = new Rect(Rect2Pos[0], Rect2Pos[1], 119, SingleRingHeight);
        //TODO: make this more dynamic instead of fixed widths and heights

        Scalar rectangleColor = new Scalar(0,0,255);

        //Draw rectangles
        Imgproc.rectangle(output, rect1, rectangleColor,2);
        Imgproc.rectangle(output, rect2, rectangleColor,2);

        //Crop YCbCr, put on lowerCrop mat
        lowerCrop = YCbCr.submat(rect1);
        upperCrop = YCbCr.submat(rect2);

        //Removing orange color
        Core.extractChannel(lowerCrop, lowerCrop, 2);
        Core.extractChannel(upperCrop, upperCrop, 2);

        //Take raw average data
        Scalar lowerAverageOrange = Core.mean(lowerCrop);
        Scalar upperAverageOrange = Core.mean(lowerCrop);

        //Take first value of average
        double finalLowerAverage = lowerAverageOrange.val[0];
        double finalUpperAverage = upperAverageOrange.val[0];

        //Compare average values
        if (finalLowerAverage > 15 && finalLowerAverage < 130 && finalUpperAverage < 130) ringCount = 4;
        else if (finalLowerAverage > 10 && finalUpperAverage < 15 && finalLowerAverage > 10 && finalUpperAverage < 15)ringCount = 0;
        else ringCount = 1;

        telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));

        return output;
    }

    public void onViewportTapped() {

        viewportPaused = !viewportPaused;

        if (viewportPaused) {
            webcam.pauseViewport();
        } else {
            webcam.resumeViewport();
        }
    }
}
