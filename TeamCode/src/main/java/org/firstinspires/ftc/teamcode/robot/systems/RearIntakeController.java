package org.firstinspires.ftc.teamcode.robot.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Controller;

@Config
public class RearIntakeController implements Controller{
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private DcMotorEx rearIntake;

    public static String ControllerName;
    public static boolean isRunning;

    /*
     * Do not change motor direction to avoid breaking odometry
     * which uses the same encoder ports
     * */

    public static double VertIntakePower = -0.4;

    public RearIntakeController(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        ControllerName = getClass().getSimpleName();

        rearIntake = hardwareMap.get(DcMotorEx.class, "rear_intake");

        rearIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void init() {
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
        isRunning = false;
        rearIntake.setPower(0);
    }

    public void run(DcMotorEx.Direction Direction) {
        isRunning = true;

        if (Direction == DcMotorEx.Direction.FORWARD) rearIntake.setPower(VertIntakePower);
        else rearIntake.setPower(-VertIntakePower);

        telemetry.addData(ControllerName, "Rear Intake Running");
    }
}
