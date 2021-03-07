package org.firstinspires.ftc.teamcode.robot.camera.algorithms;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class VerticalRingDetector extends OpenCvPipeline {

    private Telemetry telemetry;
    private boolean debug = false;

    private static final Scalar lowerOrange = new Scalar(0.0, 141.0, 0.0);
    private static final Scalar upperOrange = new Scalar(255.0, 230.0, 95.0);

    private int CAMERA_WIDTH = 640;
    private static double MIN_ASPECT_RATIO = 2;

    private double centerX;
    private double centerY;
    private static double aspectRatio = 2;
    private static double ringWidth = 20;

    public VerticalRingDetector(Telemetry telemetry, boolean debug) {
        this.telemetry = telemetry;
        this.debug = debug;
    }

    public VerticalRingDetector(Telemetry telemetry) {
        this(telemetry, false);
    }

    public double getDisplacementFromCenter() {
        return centerX - getCameraCenterX();
    }

    public double getCameraCenterX() {
        return (double)CAMERA_WIDTH / 2;
    }

    public static double getAspectRatio() { return aspectRatio; }
    public static double getRingWidth() { return ringWidth; }

    @Override
    public Mat processFrame(Mat input) {
        CAMERA_WIDTH = input.width();

        Mat ret = new Mat();
        Mat mat = new Mat();
        Mat hierarchy = new Mat();
        Mat mask = null;

        try { // try catch in order for opMode to not crash and force a restart
            /**converting from RGB color space to YCrCb color space**/
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);

            /**isolate colors in selected alliance color range**/
            mask = new Mat(mat.rows(), mat.cols(), CvType.CV_8UC1); // variable to store mask in, 1 channel
            Core.inRange(mat, lowerOrange, upperOrange, mask);

            /**applying to input and putting it on ret in black or yellow**/
            Core.bitwise_and(input, input, ret, mask);

            /**applying GaussianBlur to reduce noise when finding contours**/
            Imgproc.GaussianBlur(mask, mask, new Size(5.0, 15.0), 0.00);

            /**finding contours on mask**/
            List<MatOfPoint> contours = new ArrayList();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);

            /**drawing contours to ret in green**/
            Imgproc.drawContours(ret, contours, -1, new Scalar(0.0, 255.0, 0.0), 3);

            /**finding widths of each contour, comparing, and storing the widest**/
            double minWidth = 20;
            Rect maxRect = new Rect();
            for (MatOfPoint c : contours) {
                MatOfPoint2f copy = new MatOfPoint2f(c.toArray());
                Rect rect = Imgproc.boundingRect(copy);

                double w = rect.width;
                double aspectRatio = (double)rect.height / (double)rect.width;

                if (w > minWidth && aspectRatio > MIN_ASPECT_RATIO) {
                    minWidth = w;
                    maxRect = rect;
                }

                c.release(); // releasing the buffer of the contour, since after use, it is no longer needed
                copy.release(); // releasing the buffer of the copy of the contour, since after use, it is no longer needed
            }

            /**drawing widest bounding rectangle to ret**/
            Scalar color = new Scalar(0,255,255);
            Imgproc.rectangle(ret, maxRect, color, 5);

            ringWidth = maxRect.width;
            centerX = maxRect.x + (double)maxRect.width/2;
            centerY = maxRect.y + (double)maxRect.height/2;

            Point center = new Point(centerX, centerY);
            Imgproc.drawMarker(ret, center, color, 0, 35);

            Imgproc.line(
                    ret,
                    new Point(CAMERA_WIDTH/2, 0),
                    new Point(CAMERA_WIDTH/2, input.height()),
                    new Scalar(255.0, .0, 255.0));

            if (debug) telemetry.addData("Vision: maxW", minWidth);
            aspectRatio = (double)maxRect.height / (double)maxRect.width;
            if(debug) telemetry.addData("Vision: Aspect Ratio", aspectRatio);

            /**returns the contour mask combined with original image for context**/
            Core.addWeighted(ret, 0.65, input, 0.35, 0, input);

        } catch (Exception e) {
            /**error handling, prints stack trace for specific debug**/
            telemetry.addData("[ERROR]", e.toString());
            telemetry.addData("[ERROR]", e.getStackTrace().toString());
        }
        finally {
            // releasing all mats after use
            ret.release();
            mat.release();
            if (mask != null) {
                mask.release();
            }
            hierarchy.release();
        }

        return input;
    }
}
