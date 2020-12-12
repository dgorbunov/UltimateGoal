package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.teamcode.opmodes.auto.sequence.BlueLeftSequence;
import org.firstinspires.ftc.teamcode.opmodes.auto.sequence.BlueRightSequence;
import org.firstinspires.ftc.teamcode.opmodes.auto.sequence.RedLeftSequence;
import org.firstinspires.ftc.teamcode.opmodes.auto.sequence.RedRightSequence;
import org.firstinspires.ftc.teamcode.opmodes.auto.sequence.Sequence;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;
import org.firstinspires.ftc.teamcode.robot.camera.CameraController;
import org.firstinspires.ftc.teamcode.robot.drive.DriveLocalizationController;
import org.firstinspires.ftc.teamcode.robot.drive.params.ThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.robot.drive.params.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.robot.systems.BumperController;
import org.firstinspires.ftc.teamcode.robot.systems.HubController;
import org.firstinspires.ftc.teamcode.robot.systems.IntakeController;
import org.firstinspires.ftc.teamcode.robot.systems.ShooterController;
import org.firstinspires.ftc.teamcode.robot.systems.WobbleController;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

@Disabled // will not show up on driver station
@Autonomous(name="FullAuto", group="Iterative Opmode")
@Config //for FTCDash
public class FullAuto extends OpMode {

    // Maps case-insensitive name to a sequence
    private Map<String, Sequence> sequences = new HashMap<>();
    private ControllerManager controllers;
    private int ringCount = -1;
    private Sequence currentSequence;
    protected final static Object lock = new Object();
    private String startingSide;
    protected String sequenceName;

    @Override
    public void init() {
        telemetry.addLine("Initializing...");

        if (startingSide.equals(FieldConstants.LeftSide)) {
            CameraController.WebcamName = CameraController.WEBCAM_LEFT;
        }
        else if (startingSide.equals(FieldConstants.RightSide)) {
            CameraController.WebcamName = CameraController.WEBCAM_RIGHT;
        } else telemetry.addLine("Could not find sequence side");

        controllers = new ControllerManager(telemetry);
        makeControllers();

        // The controllers must be set up before
        makeSequences();

        // Initialize all controllers
        controllers.init();

        synchronized (lock) {
            currentSequence = getSequence(sequenceName);
            if (currentSequence == null) {
                telemetry.addLine("No sequence found!");
                return;
            }
        }

        telemetry.addLine("Initialized");
    }

    @Override
    public void init_loop() {
        computeRingCount();
    }

    @Override
    public void start() { //code to run once when play is hit
        controllers.start(); //stop camera instance

            try {
                telemetry.addLine("Initializing sequence: " + getSequenceName(currentSequence));
                currentSequence.init(ringCount);

                telemetry.addLine("Executing sequence: " + getSequenceName(currentSequence));

                // execute runs async
                currentSequence.execute();
            } catch (Exception e) {
                telemetry.addLine("Exception while executing sequence: " + e.toString());
            }
    }

    @Override
    public void loop() {
    }

    @Override
    public void stop() {
        synchronized (lock) {
            if (currentSequence != null) currentSequence.stop();
            controllers.stop(); //TODO: stop drivetrain with mode = idle
        }
    }

    public String getSequence() {
        synchronized (lock) {
            return this.sequenceName;
        }
    }

    public void SetSequenceName(String sequenceName) {
        synchronized (lock) {
            this.sequenceName = sequenceName;
        }
    }

    protected String makeSequenceName(String alliance, String side) {
        startingSide = side;
        return (alliance + side).toLowerCase();
    }

    private void makeControllers() {
        controllers.add(FieldConstants.Camera, new CameraController(hardwareMap, telemetry));
        controllers.add(FieldConstants.Drive, new DriveLocalizationController(hardwareMap, telemetry));
        controllers.add(FieldConstants.Intake, new IntakeController(hardwareMap, telemetry));
        controllers.add(FieldConstants.Shooter, new ShooterController(hardwareMap, telemetry));
        controllers.add(FieldConstants.Wobble, new WobbleController(hardwareMap, telemetry));
        controllers.add(FieldConstants.Hub, new HubController(hardwareMap, telemetry));
        controllers.add(FieldConstants.Bumper, new BumperController(hardwareMap, telemetry));
    }

    private void makeSequences() {
        synchronized (lock) {
            sequences.put(makeSequenceName(
                    FieldConstants.RedAlliance, FieldConstants.LeftSide),
                    new RedLeftSequence(controllers, telemetry));

            sequences.put(makeSequenceName(
                    FieldConstants.RedAlliance, FieldConstants.RightSide),
                    new RedRightSequence(controllers, telemetry));

            sequences.put(makeSequenceName(
                    FieldConstants.BlueAlliance, FieldConstants.LeftSide),
                    new BlueLeftSequence(controllers, telemetry));

            sequences.put(makeSequenceName(
                    FieldConstants.BlueAlliance, FieldConstants.RightSide),
                    new BlueRightSequence(controllers, telemetry));
        }
    }

    private Sequence getSequence(String name) {
        return sequences.get(name.toLowerCase());
    }

    private String getSequenceName(Sequence sequence){
        return sequence.getClass().getSimpleName();
    }

    private void computeRingCount() {
        CameraController camera = controllers.get(CameraController.class, FieldConstants.Camera);
        if (camera == null) {
            telemetry.addLine("Camera not initialized");
            return;
        }

        String strNumRings = camera.rankRings();
        telemetry.addLine("Camera rankRings returned: " + strNumRings);

//        int ringCountOpenCV = camera.countRingsOpenCV();
//        telemetry.addData("OpenCV returned rings", ringCountOpenCV);

        int rings = camera.ringsToInt(strNumRings);
        synchronized (lock) {
            ringCount = Optional.ofNullable(rings).orElse(-1);
            telemetry.addData("Camera ringsToInt returned: ", ringCount);
        }
    }
}
