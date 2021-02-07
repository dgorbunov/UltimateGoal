package org.firstinspires.ftc.teamcode.robot.camera.algorithms;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;
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

public class WobbleDetector extends OpenCvPipeline {

    private Telemetry telemetry;
    private boolean debug = false;
    private FieldConstants.Alliance alliance; //Blue or Red

    //TODO: Calibrate colors
    private static Scalar lowerRed = new Scalar(35, 170, 90);
    private static Scalar upperRed = new Scalar(150, 255, 110);
    private static Scalar lowerBlue = new Scalar(0.0, 141.0, 0.0);
    private static Scalar upperBlue = new Scalar(255.0, 230.0, 95.0);

    private int CAMERA_WIDTH = 800;
    private static double HORIZON = 0; //(100.0 / 320.0) * CAMERA_WIDTH;
    private static double MAX_CONTOUR_WIDTH = 80; // (50.0 / 320.0) * CAMERA_WIDTH
    private static double MIN_ASPECT_RATIO = 1.2;

    private double centerX;
    private double centerY;

    public WobbleDetector(Telemetry telemetry, boolean debug, FieldConstants.Alliance alliance) {
        this.telemetry = telemetry;
        this.alliance = alliance;
        this.debug = debug;

//        upperRed = ColorConverter.convert(upperRed, Imgproc.COLOR_RGB2YCrCb);
//        lowerRed = ColorConverter.convert(lowerRed, Imgproc.COLOR_RGB2YCrCb);
//        upperBlue = ColorConverter.convert(upperBlue, Imgproc.COLOR_RGB2YCrCb);
//        lowerBlue = ColorConverter.convert(lowerBlue, Imgproc.COLOR_RGB2YCrCb);
        telemetry.addData("upperRed", upperRed.toString());
        telemetry.addData("upperBlue", upperBlue.toString());
        telemetry.addData("lowerRed", lowerRed.toString());
        telemetry.addData("lowerBlue", lowerBlue.toString());
        /*
        Log/d prints:
        [182.0, 52.0, 73.0, 0.0] upperRed, RGB: 255, 60, 60
        [161.0, 171.0, 69.0, 0.0] upperBlue, RGB: 255.0, 230.0, 95.0
        [140.0, 90.0, 113.0, 0.0] lowerRed, RGB: 150, 0, 0
        [75.0, 134.0, 132.0, 0.0] lowerBlue, RGB: 0.0, 141.0, 0.0
         */
        RobotLog.d(upperRed.toString());
        RobotLog.d(upperBlue.toString());
        RobotLog.d(lowerRed.toString());
        RobotLog.d(lowerBlue.toString());
    }

    public WobbleDetector(Telemetry telemetry, FieldConstants.Alliance alliance) {
        this(telemetry, false, alliance);
    }

    public double getDisplacementFromCenter() {
        return centerX - getCameraCenterX();
    }

    public double getCameraCenterX() {
        return (double)CAMERA_WIDTH/2;
    }

    @Override
    public Mat processFrame(Mat input) {
        CAMERA_WIDTH = input.width();

        Mat ret = new Mat();
        Mat mat = new Mat();
        try { // try catch in order for opMode to not crash and force a restart
            /**converting from RGB color space to YCrCb color space**/
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);

            /**isolate colors in selected alliance color range**/
            Mat mask = new Mat(mat.rows(), mat.cols(), CvType.CV_8UC1); // variable to store mask in, 1 channel
            if (alliance == FieldConstants.Alliance.Red) {
                Core.inRange(mat, lowerRed, upperRed, mask);
            } else Core.inRange(mat, lowerBlue, upperBlue, mask);

            /**applying to input and putting it on ret in black or yellow**/
            Core.bitwise_and(input, input, ret, mask);

            /**applying GaussianBlur to reduce noise when finding contours**/
            Imgproc.GaussianBlur(mask, mask, new Size(5.0, 15.0), 0.00);

            /**finding contours on mask**/
            List<MatOfPoint> contours = new ArrayList();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);

            /**drawing contours to ret in green**/
            Imgproc.drawContours(ret, contours, -1, new Scalar(0.0, 255.0, 0.0), 3);

            /**finding widths of each contour, comparing, and storing the widest**/
            double maxWidth = 0;
            Rect maxRect = new Rect();
            for (MatOfPoint c : contours) {
                MatOfPoint2f copy = new MatOfPoint2f(c.toArray());
                Rect rect = Imgproc.boundingRect(copy);

                double w = rect.width;
                double aspectRatio = (double)rect.height / (double)rect.width;

                //TODO: switch to min, but within a reasonable range to remove noise
                if (w > maxWidth && aspectRatio > MIN_ASPECT_RATIO) {
                    maxWidth = w;
                    maxRect = rect;
                }

                c.release(); // releasing the buffer of the contour, since after use, it is no longer needed
                copy.release(); // releasing the buffer of the copy of the contour, since after use, it is no longer needed
            }

            /**drawing widest bounding rectangle to ret**/
            Scalar color;
            if (alliance == FieldConstants.Alliance.Red) color = new Scalar(0,0,255);
            else color = new Scalar(255,0,0);
            Imgproc.rectangle(ret, maxRect, color, 5);

            centerX = maxRect.x + (double)maxRect.width/2;
            centerY = maxRect.y + (double)maxRect.height/2;

            Point center = new Point(centerX, centerY);
            Imgproc.drawMarker(ret, center, color, 0, 35);

            if (debug) telemetry.addData("Vision: maxW", maxWidth);

            /** checking if widest width is LESS than equal to maximum width
             * using Java ternary expression to set height variable
             *
             **/

            //TODO: use aspect ratio to calculate probable location
            double aspectRatio = (double)maxRect.height / (double)maxRect.width;
            if(debug) telemetry.addData("Vision: Aspect Ratio", aspectRatio);

            // releasing all mats after use
            mat.release();
            mask.release();
            hierarchy.release();

        } catch (Exception e) {
            /**error handling, prints stack trace for specific debug**/
            telemetry.addData("[ERROR]", e.toString());
            telemetry.addData("[ERROR]", e.getStackTrace().toString());

        }

        /**returns the contour mask combined with original image for context**/
        Mat output = new Mat();
        Core.addWeighted(ret, 0.65, input, 0.35, 0, output);
        return output;
    }
}
