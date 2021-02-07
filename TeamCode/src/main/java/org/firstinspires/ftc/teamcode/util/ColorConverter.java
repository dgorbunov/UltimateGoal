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

    public static Scalar ConvertColor(Scalar input, int code) {
        Mat in = new Mat(1,1, CV_8UC3, input);
        Mat out = new Mat();

        Imgproc.cvtColor(in, out, code);

        return new Scalar(out.get(1,1)[0], out.get(1,1)[1], out.get(1,1)[2]);
    }
//
//    public static Scalar RGB2YCrCb(Scalar input) {
//        //sample input and output
//        float R = (float)input.val[0];
//        float G = (float)input.val[1];
//        float B = (float)input.val[2];
//
//        //https://stackoverflow.com/questions/29688548/converting-rgb-to-ycbcr-in-opencv
//        //https://docs.opencv.org/3.4/de/d25/imgproc_color_conversions.html
//        //actual conversion from RGB to YCrCb
//        float delta = 0.5f;
//        float Y  = 0.299f * R + 0.587f * G + 0.114f * B;
//        float Cb = (B - Y) * 0.564f + delta;
//        float Cr = (R - Y) * 0.713f + delta;
//
//        if (Y > 255 || Cr > 255 || Cb > 255) {
//            try {
//                throw new Exception("YCrCb values over limit");
//            } catch (Exception e) {
//                e.printStackTrace();
//            }
//        }
//
//        return new Scalar(Y, Cr, Cb);
//    }
//

}
