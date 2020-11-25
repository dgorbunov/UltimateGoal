package org.firstinspires.ftc.teamcode.robot.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Controller;
import org.firstinspires.ftc.teamcode.util.MockDcMotorEx;

public class ShooterController implements Controller {
    Telemetry telemetry;
    MockDcMotorEx shooter;
    float wheelRadius = 0.051f; //meters

    public ShooterController (HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        shooter = new MockDcMotorEx("shooter", telemetry);
    }

    @Override
    public void init() {
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void shoot(Gamepad gamepad, boolean button){
        if (button) shooter.setPower(1); //shooter.setVelocity();
         else stop();

        String velocity = shooter.getVelocity(AngleUnit.RADIANS) + " rad/s";
        String velocityTangential = shooter.getVelocity(AngleUnit.RADIANS) * wheelRadius + " m/s";
//        telemetry.addData("Shooter Velocity:", velocity);
//        telemetry.addData("Shooter Velocity:", velocityTangential); //v = r*w
    }

    @Override
    public void start() {
    }

    @Override
    public void stop() {
        shooter.setPower(0);
    }
}
