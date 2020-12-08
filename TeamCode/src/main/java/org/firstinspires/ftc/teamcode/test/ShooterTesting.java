package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
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

@TeleOp(name="ShooterTesting", group="Iterative Opmode")
public class ShooterTesting extends OpMode {

    public static volatile double MotorRPM = 5280;
    private static final AngleUnit unit = AngleUnit.RADIANS;
    private static final double MotorVelocity = (MotorRPM * 2 * Math.PI) / 60; //x rev/min * 2pi = x rad/min / 60 = x rad/sec

    DcMotorEx shooter = hardwareMap.get(DcMotorEx .class, "shooter");
    Servo bumper = hardwareMap.get(Servo.class, "arm");
    ControllerManager controllers;
    HubController hub;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

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
        shooter.setVelocity(MotorVelocity, unit);

        double velocity = shooter.getVelocity();
        telemetry.addData("target velocity (rad/s)", MotorVelocity);
        telemetry.addData("velocity (rad/s)", velocity);
        dashboardTelemetry.addData("target velocity (rad/s)", MotorVelocity);
        dashboardTelemetry.addData("velocity (rad/s)", velocity);

        String currentDraw = hub.getFormattedCurrentDraw();
        telemetry.addLine(currentDraw);
        dashboardTelemetry.addLine(currentDraw);

        if (gamepad1.a) bumper.setPosition(0.6);
        if (gamepad1.b) bumper.setPosition(0.4);
    }

    @Override
    public void stop() {
        shooter.setVelocity(0);
        controllers.stop();
    }
}
