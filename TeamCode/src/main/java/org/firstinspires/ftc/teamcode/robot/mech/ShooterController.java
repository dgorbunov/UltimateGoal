package org.firstinspires.ftc.teamcode.robot.mech;

import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.opmodes.tele.MainTele;
import org.firstinspires.ftc.teamcode.robot.Controller;

public class ShooterController implements Controller {

    float wheelRadius = 0.051f; //meters

    Telemetry telemetry = MainTele.GetTelemetry();
    DcMotorEx shooter;

    public ShooterController (DcMotor shooter,
                              HardwareMap hardwareMap){

    }

    @Override
    public void init() {
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void shoot(Gamepad gamepad, boolean button){
        if (button){
            shooter.setPower(1);
        } else stop();

        String velocity = shooter.getVelocity(AngleUnit.RADIANS) + " rad/s";
        String velocityTangential = shooter.getVelocity(AngleUnit.RADIANS) * wheelRadius + " m/s";
        telemetry.addData("Shooter Velocity:", velocity);
        telemetry.addData("Shooter Velocity:", velocityTangential); //v = r*w
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
        shooter.setPower(0);
    }
}
