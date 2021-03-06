package org.firstinspires.ftc.teamcode.robot.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Controller;

@Config
public class VertIntakeController implements Controller{
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private DcMotorEx vIntake;
    private CRServo wheel1;
    private CRServo wheel2;

    public static String ControllerName;
    public static boolean isRunning;

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
        wheel1 = hardwareMap.get(CRServo.class, "vIntakeWheel1");
        wheel2 = hardwareMap.get(CRServo.class, "vIntakeWheel2");

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
        isRunning = false;

        vIntake.setPower(0);
        wheel1.setPower(0);
        wheel2.setPower(0);

        IntakeController.stopSweeper();
    }

    public void run(DcMotorEx.Direction Direction) {
        isRunning = true;
        IntakeController.startSweeper();

        if (Direction == DcMotorEx.Direction.FORWARD) vIntake.setPower(VertIntakePower);
        else vIntake.setPower(-VertIntakePower);

        wheel1.setPower(1);
        wheel2.setPower(1);
        telemetry.addData(ControllerName, "V. Intake Running");
    }
}
