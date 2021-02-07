package org.firstinspires.ftc.teamcode.robot.camera.algorithms;


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

import static org.firstinspires.ftc.teamcode.robot.camera.algorithms.RingDetector.ringNames.FOUR;

public class RingDetector extends OpenCvPipeline {

    private Telemetry telemetry;
    private boolean debug = false;

    private static Scalar lowerOrange = new Scalar(0.0, 141.0, 0.0);
    private static Scalar upperOrange = new Scalar(255.0, 230.0, 95.0);

    private int CAMERA_WIDTH = 720;
    private static double HORIZON = 200; //(100.0 / 320.0) * CAMERA_WIDTH;
    private static double MIN_CONTOUR_WIDTH = 80; // (50.0 / 320.0) * CAMERA_WIDTH
    private static double ASPECT_RATIO_THRES = 0.7;

    private int ringCount = 0;
    private String ringCountStr = "";

    public static enum ringNames {
        NONE, ONE, FOUR
    }

    public RingDetector(Telemetry telemetry, boolean debug) {
        this.telemetry = telemetry;
        this.debug = debug;
    }

    public RingDetector(Telemetry telemetry) {
        this(telemetry, false);
    }

    public int getRingCount() {
        return ringCount;
    }

    public String getRingCountStr() {
        return ringCountStr;
    }

    @Override
    public Mat processFrame(Mat input) {
        CAMERA_WIDTH = input.width();
        // ret.release(); // releasing mat to release backing buffer
        // must release at the start of function since this is the variable being returned

        Mat ret = new Mat();
        Mat mat = new Mat(); // resetting pointer held in ret
        try { // try catch in order for opMode to not crash and force a restart
            /**converting from RGB color space to YCrCb color space**/
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

            /**checking if any pixel is within the orange bounds to make a black and white mask**/
            Mat mask = new Mat(mat.rows(), mat.cols(), CvType.CV_8UC1); // variable to store mask in
            Core.inRange(mat, lowerOrange, upperOrange, mask);

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
                // checking if the rectangle is below the horizon
                if (w > maxWidth && rect.y + rect.height > HORIZON) {
                    maxWidth = w;
                    maxRect = rect;
                }
                c.release(); // releasing the buffer of the contour, since after use, it is no longer needed
                copy.release(); // releasing the buffer of the copy of the contour, since after use, it is no longer needed
            }

            /**drawing widest bounding rectangle to ret in blue**/
            Imgproc.rectangle(ret, maxRect, new Scalar(0.0, 0.0, 255.0), 4);

            /** drawing a red line to show the horizon (any above the horizon is not checked to be a ring stack **/
            Imgproc.line(
                    ret,
                    new Point(.0, HORIZON),
                    new Point(CAMERA_WIDTH, HORIZON),
                    new Scalar(255.0, .0, 255.0));

            if (debug) telemetry.addData("Vision: maxW", maxWidth);

            /** checking if widest width is greater than equal to minimum width
             * using Java ternary expression to set height variable
             *
             * height = maxWidth >= MIN_WIDTH ? aspectRatio > BOUND_RATIO ? FOUR : ONE : ZERO
             **/

            if (maxWidth >= MIN_CONTOUR_WIDTH) {
                double aspectRatio = (double)maxRect.height / (double)maxRect.width;

                if(debug) telemetry.addData("Vision: Aspect Ratio", aspectRatio);

                /** checks if aspectRatio is greater than ASPECT_RATIO_THRES
                 * to determine whether stack is ONE or FOUR
                 */
                if (aspectRatio > ASPECT_RATIO_THRES) {
                    ringCount = 4; // FOUR
                    ringCountStr = FOUR.toString();
                }
                else {
                    ringCount = 1; // ONE
                    ringCountStr = ringNames.ONE.toString();
                }
            } else {
                ringCount = 0; // ZERO
                ringCountStr = ringNames.NONE.toString();
            }

            if (debug) telemetry.addData("Vision: Height", ringCountStr);

            // releasing all mats after use
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
        mat.release();
        ret.release();

        return output;
    }
}
