package org.firstinspires.ftc.teamcode.robot.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Controller;
import org.firstinspires.ftc.teamcode.util.MockDcMotor;
import org.firstinspires.ftc.teamcode.util.MockDcMotorEx;

import static org.firstinspires.ftc.teamcode.util.Sleep.sleep;

public class VIntakeController implements Controller {
    private Telemetry telemetry;
    private MockDcMotorEx vIntake;
    public static String ControllerName;

    public VIntakeController(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        vIntake = new MockDcMotorEx("v_intake", telemetry);
        ControllerName = getClass().getSimpleName();
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

    public void run(double motorPower) {
        telemetry.addData(ControllerName, "V. Intake Running");
        vIntake.setPower(motorPower);
    }
}
