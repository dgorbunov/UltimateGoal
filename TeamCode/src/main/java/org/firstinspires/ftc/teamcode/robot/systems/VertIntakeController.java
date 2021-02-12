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

    /*
     * Do not change motor direction to avoid breaking odometry
     * which uses the same encoder ports
     * */

    public static double VertIntakePower = -0.4;

    public VertIntakeController(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        ControllerName = getClass().getSimpleName();

        vIntake = hardwareMap.get(DcMotorEx.class, "vIntake");

        vIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    public void run(DcMotorEx.Direction Direction) {
        if (Direction == DcMotorSimple.Direction.FORWARD) vIntake.setPower(VertIntakePower);
        else vIntake.setPower(-VertIntakePower);
        telemetry.addData(ControllerName, "V. Intake Running");
    }
}
