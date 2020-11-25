package org.firstinspires.ftc.teamcode.robot.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Controller;
import org.firstinspires.ftc.teamcode.util.MockServo;

public class WobbleController implements Controller {
    Telemetry telemetry;

    MockServo wobbleGrip;
    MockServo wobbleLift;

    public WobbleController(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        wobbleGrip = new MockServo("wobbleGrip", telemetry);
        wobbleLift = new MockServo("wbbbleLift", telemetry);
    }

    @Override
    public void init() {
//        wobbleGrip.scaleRange();
//        wobbleLift.scaleRange();
//        //TODO: Scale servo range
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
