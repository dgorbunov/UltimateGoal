package org.firstinspires.ftc.teamcode.robot.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Controller;
@Config
public class BumperController implements Controller {

    private Servo bumper;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    public static double BumpPosition = 0.6;
    public static double RetractPosition = 0.35;

    public BumperController(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        //TODO: migrate to @ShooterController
    }

    @Override
    public void init() {
        bumper = hardwareMap.get(Servo.class, "bumper");
        retract();
    }

    @Override
    public void start() {

    }

    public void bump() {
        bumper.setPosition(BumpPosition);
    }

    public void retract() {
        bumper.setPosition(RetractPosition);
    }

    @Override
    public void stop() {
        bumper.setPosition(RetractPosition);
    }
}
