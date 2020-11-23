package org.firstinspires.ftc.teamcode.robot.mech;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Controller;

public class WobbleController implements Controller {

    Telemetry telemetry;

    public WobbleController(HardwareMap hardwareMap, Telemetry tel) {
        this.telemetry = tel;
    }

    @Override
    public void init() {

    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {

    }

    // TODO: drop wobble
    public void drop() {
    }

    // TODO: pickup wobble
    public void pickup() {
    }
}
