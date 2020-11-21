package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.opmodes.auto.trajectories.TrajectoryController;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;
import org.firstinspires.ftc.teamcode.robot.camera.CameraController;
import org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController;

@Autonomous(name="fullAuto", group="Iterative Opmode")
public class fullAuto extends OpMode {

    //TODO: make seperate opmodes for starting positions
    public static String alliance = "Red"; // or "Blue"
    public String side = "Left"; //or "Right"

    private CameraController vuforia;
    private TrajectoryController trajectory;
    private ControllerManager controllers;

    public int test;

    public static int numRings;
    private static String strNumRings;

    @Override
    public void init() {
        telemetry.addLine("Initializing...");
        vuforia = new CameraController(hardwareMap, telemetry);
        trajectory = new TrajectoryController(hardwareMap, telemetry);

        controllers = new ControllerManager(vuforia, trajectory);

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

        trajectory.selectTrajectory(alliance, side, numRings);
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        controllers.stop();
    }
}
