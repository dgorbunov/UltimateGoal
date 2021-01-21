package org.firstinspires.ftc.teamcode.robot.camera;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="CameraTest", group="Iterative Opmode")
public class CameraTest extends OpMode {

    private Camera camera;

    @Override
    public void init() {
        camera = new Camera(hardwareMap, telemetry);
        camera.init();
        telemetry.addLine("Initialized");
        telemetry.update();
    }

    public void init_loop() {
        telemetry.addLine(camera.getRingCountStr());
        telemetry.update();
    }

    public void start() {
        camera.start();
        telemetry.addLine("Started");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addLine("In loop");

    }

    public void stop() {
        camera.stop();
        telemetry.addLine("Stopped");
        telemetry.update();
    }
}
