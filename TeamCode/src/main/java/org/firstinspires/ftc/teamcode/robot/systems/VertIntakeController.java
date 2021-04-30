package org.firstinspires.ftc.teamcode.robot.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Controller;

@Config
@Deprecated
public class VertIntakeController implements Controller{
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private DcMotorEx vIntake;
    private Servo floorSweeper;

    public static String ControllerName;
    public static boolean isRunning;

    /*
     * Do not change motor direction to avoid breaking odometry
     * which uses the same encoder ports
     * */

    public static double VertIntakePower = -0.4;
    public static double FloorSweeperRestPos = 0.2;
    public static double FloorSweeperOutPos = 0.8;

    public VertIntakeController(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        ControllerName = getClass().getSimpleName();

        vIntake = hardwareMap.get(DcMotorEx.class, "vIntake");
        floorSweeper = hardwareMap.get(Servo.class, "floor_sweeper");

        vIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void init() {
        sweepIn();
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
        isRunning = false;

        vIntake.setPower(0);

        IntakeController.stopSweeper();
    }

    public void run(DcMotorEx.Direction Direction) {
        isRunning = true;
        IntakeController.startSweeper();

        if (Direction == DcMotorEx.Direction.FORWARD) vIntake.setPower(VertIntakePower);
        else vIntake.setPower(-VertIntakePower);

        telemetry.addData(ControllerName, "V. Intake Running");
    }

    public void sweepOut() {
        floorSweeper.setPosition(FloorSweeperOutPos);
    }

    public void sweepIn() {
        floorSweeper.setPosition(FloorSweeperRestPos);
    }
}
