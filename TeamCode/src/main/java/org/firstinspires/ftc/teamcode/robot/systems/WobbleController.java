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
    private Servo wobbleRotate;
    public static String ControllerName;

    public static double GripGrabPos = 0.5;
    public static double GripReleasePos = 1;
    public static double ArmDropPos = 0;
    public static double ArmPickupPos = 0.5;

    public WobbleController(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        ControllerName = getClass().getSimpleName();
    }

    @Override
    public void init() {
//        wobbleGrip = hardwareMap.get(Servo.class, "wobble_grip");
//        wobbleRotate = hardwareMap.get(Servo.class, "wobble_rotate");
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
        telemetry.addData(ControllerName, "Dropping Wobble");
        sleep(1000);
    }

    // TODO: pickup wobble
    public void pickup() {
        telemetry.addData(ControllerName, "Picking Up Wobble");
        sleep(1000);
    }


    }
