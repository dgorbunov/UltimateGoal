package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.opmodes.auto.sequence.SequenceFollower;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;
import org.firstinspires.ftc.teamcode.robot.camera.CameraController;

@Autonomous(name="fullAuto", group="Iterative Opmode")
public class FullAuto extends OpMode {

    //TODO: make seperate opmodes for starting positions
    public static String alliance = "Red"; // or "Blue"
    public static String side = "Left"; //or "Right"

    private CameraController vuforia;
    private ControllerManager controllers;
    private SequenceFollower sequenceFollower;


    public static int numRings;
    private static String strNumRings;

    @Override
    public void init() {
        telemetry.addLine("Initializing...");
        vuforia = new CameraController(hardwareMap, telemetry);
        sequenceFollower = new SequenceFollower(hardwareMap, telemetry)

        controllers = new ControllerManager(vuforia);

        controllers.init();
        telemetry.addLine("Initialized");

    }

    @Override
    public void init_loop() {
        strNumRings = vuforia.rankRings();
        telemetry.addLine(strNumRings);
    }

    @Override
    public void start() { //code to run once when play is hit
        if (strNumRings.equals("None")) numRings = 0;
        else if (strNumRings.equals("Single")) numRings = 1;
        else if (strNumRings.equals("Quad")) numRings = 4;
        controllers.start(); //stop vuforia instance
        sequenceFollower.selectTrajectory(alliance, side, strNumRings, numRings);
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        controllers.stop();
    }
}
