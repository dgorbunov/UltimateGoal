package org.firstinspires.ftc.teamcode.robot.camera;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.robot.Controller;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;


public class CameraController implements Controller {

    Telemetry telemetry;
    HardwareMap hardwareMap;

    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/vision/UltimateGoal.tflite"; //For OpenRC, loaded from internal storage to reduce APK size
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY = "Aa/NlSv/////AAABmfbIZJDVPkVejecKu21N5r4cTLhAMLAnbwXd1tcQJ9MqaVnqS+4aph3k9bZBglo+YhRJ243YKUAEpsFJEzqyyqrqGMSU8c9wxzQIakH+VFLamT1m/XPCW5M40u3k/BeLk03yiovXd3wCuGWVeAI6ipHlI2h+uMY0Q+HKr8TOFljzHXlqe7wsTbDhXu7tZRDf7LTPT5eWGZRrtHe7VgRW3sFUJ+3HvauBg20E/PRwQEDtFNNFohTMEOumOiV3EUenXrYnrINqlNOhPDlBlkm2du7bHuDho2TCv11DEmHWXCE+Pz8i1tLsaS3dyfjOCwO8BwG468ZsjQiGIFU4FEFqV34W9zLYdwEpaqhCP4OkpoIz";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public CameraController(HardwareMap hwMap, Telemetry tel) {

        hardwareMap = hwMap;
        telemetry = tel;
    }

    @Override
    public void init() {
        initVuforia();
        initTfod();
    }

    @Override
    public void start() { }

    public void countRings() { //TODO: sort updatedRecognitions by probability, return top probability with # of rings
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                }
                telemetry.update();
            }
        }
    }

    @Override
    public void stop() {
        if (tfod != null) tfod.shutdown();

    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

        if (tfod != null) tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(2.5, 1.78);

    }
}
