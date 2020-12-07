package org.firstinspires.ftc.teamcode.robot.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Controller;
import org.firstinspires.ftc.teamcode.util.MockServo;

public class BumperController implements Controller {

    private MockServo bumper;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    public BumperController(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        bumper = new MockServo("bumper", telemetry);
    }

    @Override
    public void init() {

    }

    @Override
    public void start() {

    }

    public void bump() {
        bumper.setPosition(0.6);
    }
    //TODO make async and retract after x time

    public void retract() {
        bumper.setPosition(0.35);
    }

    @Override
    public void stop() {

    }
}
