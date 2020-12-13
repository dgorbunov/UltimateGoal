package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.opmodes.auto.FieldConstants;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;
import org.firstinspires.ftc.teamcode.robot.systems.BumperController;
import org.firstinspires.ftc.teamcode.robot.systems.HubController;
import org.firstinspires.ftc.teamcode.robot.systems.ShooterController;

import static org.firstinspires.ftc.teamcode.util.Sleep.sleep;

@Config
@TeleOp(name="ShooterTesting", group="Iterative Opmode")
public class ShooterTesting extends OpMode {

    /** Accessible via FTC Dashboard */
    public static volatile double MotorRPM = 5280;
    public static volatile double ShootingDelay = 750;
    public static volatile double RetractDelay = 450;
    public static volatile double BumpPosition = 0.6;
    public static volatile double RetractPosition = 0.4;

    private static final AngleUnit unit = AngleUnit.RADIANS;
    private static final double MotorVelocity = (MotorRPM * 2 * Math.PI) / 60; //x rev/min * 2pi = x rad/min / 60 = x rad/sec
    float wheelRadius = 0.051f; //meters

    DcMotorEx shooter = hardwareMap.get(DcMotorEx .class, "shooter");
    Servo bumper = hardwareMap.get(Servo.class, "arm");
    ControllerManager controllers;
    HubController hub;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    Thread shootThread = new Thread(this::shoot);
    Thread telemetryThread = new Thread(this::telemetry);
    Thread motorThread = new Thread(this::runMotor);

    @Override
    public void init() {
        controllers = new ControllerManager(telemetry);
        hub = new HubController(hardwareMap, telemetry);

        controllers.add(FieldConstants.Hub, hub);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        controllers.init();

    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start(){
        controllers.start();
    }

    @Override
    public void loop() {
        /**
         * This is multithreaded so the motor can maintain a constant velocity with setVelocity()
         * as the rings go by, lowering it's speed
         */

        motorThread.start();
        telemetryThread.start();

        if (gamepad1.a) {
            shootThread.start();
        }
    }

    private void shoot(){
        for (int i = 0; i < 3; i++) {
            bumper.setPosition(BumpPosition);
            sleep(RetractDelay);
            bumper.setPosition(RetractPosition);
            sleep(ShootingDelay - RetractDelay);
        }
    }

    private void runMotor() {
        shooter.setVelocity(MotorVelocity, unit);
    }

    private void telemetry(){
        double velocity = shooter.getVelocity(unit);
        telemetry.addData("target velocity (rad/s)", MotorVelocity);
        telemetry.addData("velocity (rad/s)", velocity);
        telemetry.addData("tangential velocity (m/s):", velocity * wheelRadius);

        dashboardTelemetry.addData("target velocity (rad/s)", MotorVelocity);
        dashboardTelemetry.addData("velocity (rad/s)", velocity);
        dashboardTelemetry.addData("tangential velocity (m/s):", velocity * wheelRadius);

        String currentDraw = hub.getFormattedCurrentDraw();
        telemetry.addLine(currentDraw);
        dashboardTelemetry.addLine(currentDraw);
    }

    @Override
    public void stop() {
        shooter.setVelocity(0);
        controllers.stop();
    }
}
