package org.firstinspires.ftc.teamcode.robot.camera;

import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.opmodes.auto.Auto;
import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;
import org.firstinspires.ftc.teamcode.robot.Controller;
import org.firstinspires.ftc.teamcode.robot.camera.algorithms.OpenCVInterface;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Config
@Deprecated
public class CameraController implements Controller {

    Telemetry telemetry;
    HardwareMap hardwareMap;
    OpenCVInterface ringDetector;

    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/vision/UltimateGoal.tflite"; //For OpenRC, loaded from internal storage to reduce APK size
    public static final String LABEL_FIRST_ELEMENT = "Quad";
    public static final String LABEL_SECOND_ELEMENT = "Single";
    public static final String LABEL_NO_ELEMENT = "None";
    public static final int INT_FIRST_ELEMENT = 4;
    public static final int INT_SECOND_ELEMENT = 1;
    public static final int INT_NO_ELEMENT = 0;
    private static final String VUFORIA_KEY = "Aa/NlSv/////AAABmfbIZJDVPkVejecKu21N5r4cTLhAMLAnbwXd1tcQJ9MqaVnqS+4aph3k9bZBglo+YhRJ243YKUAEpsFJEzqyyqrqGMSU8c9wxzQIakH+VFLamT1m/XPCW5M40u3k/BeLk03yiovXd3wCuGWVeAI6ipHlI2h+uMY0Q+HKr8TOFljzHXlqe7wsTbDhXu7tZRDf7LTPT5eWGZRrtHe7VgRW3sFUJ+3HvauBg20E/PRwQEDtFNNFohTMEOumOiV3EUenXrYnrINqlNOhPDlBlkm2du7bHuDho2TCv11DEmHWXCE+Pz8i1tLsaS3dyfjOCwO8BwG468ZsjQiGIFU4FEFqV34W9zLYdwEpaqhCP4OkpoIz";

    public static String WebcamName = "Webcam 1";
    //TODO: Remove once using one centered camera on final robot

    private VuforiaLocalizer vuforia = null;
    private OpenGLMatrix lastLocation = null;
    private TFObjectDetector tfod;

    private int tfodCropX = 425;
    private int tfodCropY = 25;

    public boolean targetVisible = false;
    private float phoneXRotate = 180;
    private float phoneYRotate  = 0;
    private float phoneZRotate  = 0;

    public final float CAMERA_FORWARD_DISPLACEMENT  = 8.5f * mmPerInch;   // eg: Camera inches in front of robot-center
    public final float CAMERA_VERTICAL_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera inches above ground
    public final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    private Recognition topRecognition = null;
    private VuforiaTrackables targetsUltimateGoal;

    public CameraController(HardwareMap hwMap, Telemetry tel) {
        hardwareMap = hwMap;
        telemetry = tel;
    }

    public void init() {
        initVuforia();
        initTfod();
        //Stream camera frames to dashboard
        FtcDashboard.getInstance().startCameraStream(tfod, 0);
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
        if (tfod != null) {
            tfod.shutdown();
        }

        if (targetsUltimateGoal != null){
            targetsUltimateGoal.deactivate();
        }

        FtcDashboard.getInstance().stopCameraStream();
    }

//    public int getNumberOfRings() {
//        // With live preview
//        WebcamName webcamName = hardwareMap.get(WebcamName.class, "NAME_OF_CAMERA_IN_CONFIG_FILE");
//        int cameraMonitorViewId = 0;
//        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
//
//        // OpenCvCamera webcam = new OpenCvCamera();
//        RingDetector detector = new RingDetector(camera, telemetry);
//        int rings = 0;
//
//        return rings;
//    }

    public void stopTFOD(){
        tfod.deactivate();
    }

    /*
     Original SDK method, not used
     */
    public void countRings() {
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
            }
        }
    }

    /*
     Sort recognitions by confidence and return highest confidence recognition
     */
    public String rankRings (){
        if (tfod != null) {
            List<Recognition> recognitions = tfod.getRecognitions();
            if (recognitions != null && recognitions.size() != 0) {
                //sort recognitions by confidence
                Collections.sort(recognitions, Comparator.comparingDouble(Recognition::getConfidence));

                //set top recognition to recognition with most confidence
                topRecognition = recognitions.get(recognitions.size() - 1);

                return topRecognition.getLabel();
            }
            else return LABEL_NO_ELEMENT;

        }
        else return "TensorFlow Not Initialized";
    }

    @Nullable
    public Integer ringsToInt(String element){
        if (element.equals(CameraController.LABEL_NO_ELEMENT)) return INT_NO_ELEMENT;
        else if (element.equals(CameraController.LABEL_FIRST_ELEMENT)) return INT_FIRST_ELEMENT;
        else if (element.equals(CameraController.LABEL_SECOND_ELEMENT)) return INT_SECOND_ELEMENT;
        else {
            telemetry.addLine("ringsToInt returned no match");
            return null;
        }
    }

    private void trackTargets(){
        // check all the trackable targets to see which one (if any) is visible.
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

    }

    @Nullable
    public Pose2d getRobotPosition(){
        trackTargets();
        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            rotation.toAngleUnit(RADIANS);
            rotation.toAxesReference(INTRINSIC);
            return new Pose2d(translation.get(0), translation.get(1), rotation.thirdAngle); //X, Y, rotation
        }
        else {
            telemetry.addData("Visible Target", "none");
            return null;
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        try {
            parameters.cameraName = hardwareMap.get(WebcamName.class, WebcamName);
        } catch(Exception e){
            telemetry.addLine("Could not initialize webcam, name not set. " + e.toString());
        }
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        initLocalizer(parameters);

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
        //

        if (Auto.getSequenceSide() != null) {
            if (Auto.getSequenceSide() == FieldConstants.LeftSide) tfod.setClippingMargins(0, tfodCropY, tfodCropX, tfodCropY);
            else tfod.setClippingMargins(tfodCropX, tfodCropY, 0, tfodCropY);
            //TODO: this was reversed because camera is upside down
        }
        tfod.setZoom(1.50, 16.0/9.0);

    }

    private void initLocalizer(VuforiaLocalizer.Parameters parameters) {

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsUltimateGoal = vuforia.loadTrackablesFromFile("/sdcard/FIRST/vision/UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");
        allTrackables.addAll(targetsUltimateGoal);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        //Set the position of the perimeter targets with relation to origin (center of field)
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        targetsUltimateGoal.activate();
    }
}