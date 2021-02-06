package org.firstinspires.ftc.teamcode.util;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import static org.opencv.core.CvType.CV_8UC3;

public class ScalarConverter {

    /**
     * Convert RGB to YCrCb
     * @param input RGB mat
     * @return YCrCb mat
     */

    public static Scalar RGB2YCrCb(Scalar input) {
        Mat in = new Mat(1,1, CV_8UC3, input);
        Mat out = new Mat();

        Imgproc.cvtColor(in, out, Imgproc.COLOR_RGB2YCrCb);

        return new Scalar(out.get(0,0)[0], out.get(0,0)[1], out.get(0,0)[2]);
    }


}
