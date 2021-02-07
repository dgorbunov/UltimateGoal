package org.firstinspires.ftc.teamcode.util;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import static org.opencv.core.CvType.CV_8UC3;

public class ColorConverter {

    /**
     * Convert RGB to YCrCb
     * @param input RGB mat
     * @return YCrCb mat
     */

    public static Scalar convert(Scalar input, int code) {
        Mat in = new Mat(1,1, CV_8UC3, input);
        Mat out = new Mat();

        Imgproc.cvtColor(in, out, code);

        double[] value = out.get(0,0);
        Scalar res = new Scalar(value[0], value[1], value[2]);
        in.release();
        out.release();

        return res;
    }
}
