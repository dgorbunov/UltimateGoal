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
    public static String ControllerName;
    private BumperController bumper;

    public static volatile double MotorRPM = 5280;
    private static final AngleUnit unit = AngleUnit.RADIANS;
    private static final double MotorVelocity = (MotorRPM * 2 * Math.PI) / 60; //x rev/min * 2pi = x rad/min / 60 = x rad/sec

    private final float wheelRadius = 0.051f; //meters
    private final int spinUpTime = 750; //ms
    private final int shootingDelay = 800; //ms
    private final int bumpDelay = 75; //ms

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

    public void shoot(int ringCount) {
        setVelocity(MotorRPM);
        sleep(spinUpTime);
        for(int i = 0; i < ringCount; i++) {
            bumper.bump();
            sleep(bumpDelay);
            bumper.retract();
            sleep(shootingDelay - bumpDelay);
        }
    }

    public void setVelocity(double velocity){
        shooter.setVelocity(MotorVelocity, unit);
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
