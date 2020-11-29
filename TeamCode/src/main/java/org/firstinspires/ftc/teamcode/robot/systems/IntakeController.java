package org.firstinspires.ftc.teamcode.robot.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Controller;
import org.firstinspires.ftc.teamcode.util.MockDcMotor;
import org.firstinspires.ftc.teamcode.util.MockDcMotorEx;

import static org.firstinspires.ftc.teamcode.util.Sleep.sleep;

public class IntakeController implements Controller {
    private Telemetry telemetry;
    private MockDcMotorEx intake;
    public static String ControllerName;

    public IntakeController(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        intake = new MockDcMotorEx("intake", telemetry);
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
        intake.setPower(0);
    }

    public void run(double motorPower, int numRings) {
        telemetry.addData(ControllerName, "Intaking");
        intake.setPower(motorPower);
        sleep(1000);
        intake.setPower(0); //TODO: add sleep dependent on numRings
    }
}
