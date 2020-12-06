package org.firstinspires.ftc.teamcode.robot.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Controller;

public class BumperController implements Controller {

    private Servo bumper;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    public BumperController(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    @Override
    public void init() {
        bumper = hardwareMap.get(Servo.class, "arm");
    }

    @Override
    public void start() {

    }

    public void bump() {
        bumper.setPosition(0.6);
    }

    public void retract() {
        bumper.setPosition(0.35);
    }

    @Override
    public void stop() {

    }
}
