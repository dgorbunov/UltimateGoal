package org.firstinspires.ftc.teamcode.robot.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Controller;

import static org.firstinspires.ftc.teamcode.util.Sleep.sleep;

@Config
public class WobbleController implements Controller {
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private Servo wobbleGrip;
    private Servo wobbleArm;
    public static String ControllerName;

    public static double GripInitPos = 0.55;
    public static double ArmInitPos = 0.85;
    public static double GripGrabPos = 0.9;
    public static double GripReleasePos = 0.55;
    public static double ArmDropPos = 0.18;
    public static double ArmPickupPos = 0.5;

    public WobbleController(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        ControllerName = getClass().getSimpleName();
    }

    @Override
    public void init() {
        wobbleGrip = hardwareMap.get(Servo.class, "wobble_grip");
        wobbleArm = hardwareMap.get(Servo.class, "wobble_arm");
//        wobbleGrip.scaleRange();
//        wobbleArm.scaleRange();
        wobbleGrip.setPosition(GripInitPos);
        wobbleArm.setPosition(ArmInitPos);
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
        wobbleGrip.setPosition(GripInitPos);
        wobbleArm.setPosition(ArmInitPos);
    }

    // TODO: drop wobble
    public void dropAuto() {
        telemetry.addData(ControllerName, "Dropping Wobble");
        wobbleArm.setPosition(ArmDropPos);
        sleep(1000);
        wobbleGrip.setPosition(ArmPickupPos);
    }

    // TODO: pickup wobble
    public void pickupAuto() {
        telemetry.addData(ControllerName, "Picking Up Wobble");
        wobbleGrip.setPosition(GripGrabPos);
        sleep(250);
        wobbleArm.setPosition(ArmPickupPos);
        sleep(750);
    }

    public void grab(){
        wobbleGrip.setPosition(GripGrabPos);
    }
    public void release(){
        wobbleGrip.setPosition(GripReleasePos);
    }
    public void lift(){
        wobbleArm.setPosition(ArmPickupPos);
    }
    public void drop(){
        wobbleArm.setPosition(ArmDropPos);
    }


    }
