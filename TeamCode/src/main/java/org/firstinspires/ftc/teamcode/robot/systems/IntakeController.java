package org.firstinspires.ftc.teamcode.robot.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Controller;

@Config
public class IntakeController implements Controller {
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private DcMotorEx intake;
    private Servo arm;
    public static String ControllerName;

    public static double ArmStartPos = 0;
    public static double ArmDropPos = 1;
    public static double IntakePower = 0.8;
    public static DcMotorEx.Direction Direction = DcMotorEx.Direction.FORWARD;

    public IntakeController(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

//        arm.scaleRange();
        ControllerName = getClass().getSimpleName();
    }

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        arm = hardwareMap.get(Servo.class, "intake_arm");
        arm.setPosition(ArmStartPos);
        intake.setDirection(Direction);
//        arm.scaleRange();
        //TODO: scale range for all servos
    }

    @Override
    public void start() {
    }

    public void extend() {
        arm.setPosition(1);
    }

    @Override
    public void stop() {
        intake.setPower(0);
    }

    public void run(DcMotorEx.Direction Direction) {
        telemetry.addData(ControllerName, "Intaking");
        if (Direction == DcMotorSimple.Direction.FORWARD) intake.setPower(IntakePower);
        else intake.setPower(-IntakePower);
    }
}
