package org.firstinspires.ftc.teamcode.robot.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Controller;

@Config
public class VertIntakeController implements Controller{
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private DcMotorEx vIntake;
    public static String ControllerName;

    public static DcMotorEx.Direction Direction = DcMotorEx.Direction.FORWARD;
    public static double VertIntakePower;

    public VertIntakeController(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        ControllerName = getClass().getSimpleName();

        vIntake = hardwareMap.get(DcMotorEx.class, "vIntake");

        vIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void init() {
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
        vIntake.setPower(0);
    }

    public void run(DcMotorSimple.Direction Direction) {
        if (Direction == DcMotorSimple.Direction.FORWARD) vIntake.setPower(VertIntakePower);
        else vIntake.setPower(-VertIntakePower);
        telemetry.addData(ControllerName, "V. Intake Running");
        vIntake.setPower(VertIntakePower);
    }
}
