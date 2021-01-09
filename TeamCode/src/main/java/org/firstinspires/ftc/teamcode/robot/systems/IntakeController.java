package org.firstinspires.ftc.teamcode.robot.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Controller;
import org.firstinspires.ftc.teamcode.util.MockDcMotor;
import org.firstinspires.ftc.teamcode.util.MockDcMotorEx;
import org.firstinspires.ftc.teamcode.util.MockServo;

import static org.firstinspires.ftc.teamcode.util.Sleep.sleep;

public class IntakeController implements Controller {
    private Telemetry telemetry;
    private MockDcMotorEx intake;
    private Servo arm;
    public static String ControllerName;

    public IntakeController(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        intake = new MockDcMotorEx("intake", telemetry);
        arm = new MockServo("intake_arm", telemetry);
//        arm.scaleRange();
        ControllerName = getClass().getSimpleName();
    }

    @Override
    public void init() {
        arm.setPosition(0);
    }

    @Override
    public void start() {
    }

    public void extend(){
        arm.setPosition(1);
    }

    @Override
    public void stop() {

        intake.setPower(0);
    }

    public void run(double motorPower) {
        telemetry.addData(ControllerName, "Intaking");
        intake.setPower(motorPower);
    }
}
