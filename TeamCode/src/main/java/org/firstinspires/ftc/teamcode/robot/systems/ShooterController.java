package org.firstinspires.ftc.teamcode.robot.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Controller;
import org.firstinspires.ftc.teamcode.util.MockDcMotorEx;

import static org.firstinspires.ftc.teamcode.util.Sleep.sleep;

public class ShooterController implements Controller {
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private MockDcMotorEx shooter;
    private float wheelRadius = 0.051f; //meters
    public static String ControllerName;

    public ShooterController (HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        ControllerName = getClass().getSimpleName();
        shooter = new MockDcMotorEx("shooter", this.telemetry);
    }

    @Override
    public void init() {
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void shoot(Gamepad gamepad, boolean button){
        if (button) shooter.setPower(1); //shooter.setVelocity();
        else stop();

         telemetry.addData(ControllerName, "Shooting");

        String velocity = shooter.getVelocity(AngleUnit.RADIANS) + " rad/s";
        String velocityTangential = shooter.getVelocity(AngleUnit.RADIANS) * wheelRadius + " m/s";
        telemetry.addData(ControllerName,"velocity:" + velocity);
        telemetry.addData(ControllerName, "velocityTangential: " + velocityTangential); //v = r*w
    }

    public void shoot(int ms) {
        shooter.setPower(1); //shooter.setVelocity();
        sleep(ms);
        shooter.setPower(0);
    }

    public void shootWithVelocity(double velocity){
        shooter.setVelocity(velocity);
    }

    public double getVelocity(){
        return shooter.getVelocity();
    }

    @Override
    public void start() {
    }

    @Override
    public void stop() {
        shooter.setPower(0);
    }
}
