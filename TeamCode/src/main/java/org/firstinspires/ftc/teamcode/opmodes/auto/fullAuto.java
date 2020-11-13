package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.ControllerManager;
import org.firstinspires.ftc.teamcode.robot.camera.CameraController;

@Autonomous(name="fullAuto", group="Iterative Opmode")
public class fullAuto extends OpMode {

    private CameraController vuforia;

    private ControllerManager controllers;

    @Override
    public void init() {
        telemetry.addLine("Initializing...");
        vuforia = new CameraController(hardwareMap, telemetry);

        controllers = new ControllerManager(vuforia);

        controllers.init();

        telemetry.addLine("Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() { //code to run once when play is hit
        controllers.start();
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        controllers.stop();
    }
}
