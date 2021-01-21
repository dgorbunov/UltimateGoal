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

    public static double ArmStartPos = 0.262;
    public static double ArmDropPos = 0.6;
    public static double IntakePower = 0.9;
    public static DcMotorEx.Direction IntakeDirection = DcMotorEx.Direction.FORWARD;
    public static Servo.Direction ArmDirection = Servo.Direction.REVERSE;

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
        arm.setDirection(ArmDirection);

        arm.setPosition(ArmStartPos);
        intake.setDirection(IntakeDirection);

    }

    @Override
    public void start() {
        arm.setPosition(ArmStartPos);
    }

    public void extend() {
        arm.setPosition(ArmDropPos);
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
