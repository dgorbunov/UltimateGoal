package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.robocol.TelemetryMessage;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.opmodes.auto.FieldConstants;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;
import org.firstinspires.ftc.teamcode.robot.systems.BumperController;
import org.firstinspires.ftc.teamcode.robot.systems.HubController;
import org.firstinspires.ftc.teamcode.robot.systems.ShooterController;

import java.lang.annotation.Target;

import static org.firstinspires.ftc.teamcode.util.Sleep.sleep;

@Config
@TeleOp(name="ShooterTesting", group="Iterative Opmode")
public class ShooterTesting extends OpMode {

    /** Accessible via FTC Dashboard */
    public static volatile double MotorRPM = 4800;
    public static volatile double ShootingDelay = 750;
    public static volatile double BumpPosition = 0.6;
    public static volatile double RetractPosition = 0.4;
    
    public final double TicksPerRev = 28;
    
    public static volatile double TargetTicksPerSecond; //x rev/min * 2pi = x rad/min / 60 = x rad/sec;;
    float wheelRadius = 0.051f; //meters

    DcMotorEx shooter;
    Servo bumper;
    ControllerManager controllers;
    HubController hub;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    Thread shootThread = new Thread(this::shoot);
    Thread telemetryThread = new Thread(this::telemetry);
    Thread motorThread = new Thread(this::runMotor);

    @Override
    public void init(){
        dashboardTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

        dashboardTelemetry.addLine("Initializing...");
        bumper = hardwareMap.get(Servo.class, "bumper");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        controllers = new ControllerManager(dashboardTelemetry);
        hub = new HubController(hardwareMap, dashboardTelemetry);

        controllers.add(FieldConstants.Hub, hub);

        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        bumper.setPosition(RetractPosition);

        dashboardTelemetry.setMsTransmissionInterval(400);

        controllers.init();
        dashboardTelemetry.addLine("Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start(){
        TargetTicksPerSecond = MotorRPM * TicksPerRev / 60;

        dashboardTelemetry.addLine("Starting");
        controllers.start();
        dashboardTelemetry.addLine("Started");
    }

    @Override
    public void loop() {
        /**
         * This is multithreaded so the motor can maintain a constant velocity with setVelocity()
         * as the rings go by, lowering it's speed
         */

        motorThread.run();
        telemetryThread.run();

        if (gamepad1.a) {
            shootThread.run();
        }
    }

    private synchronized void shoot(){
        for (int i = 0; i < 3; i++) {
            bumper.setPosition(BumpPosition);
            bumper.setPosition(RetractPosition);
            sleep(ShootingDelay);
        }
    }

    private synchronized void runMotor() {
        dashboardTelemetry.addLine("Motor is running");
        shooter.setVelocity(TargetTicksPerSecond);
    }

    private synchronized void telemetry(){
        double velocity = shooter.getVelocity();
        double RPM = velocity / TicksPerRev * 60;
        double velocityRad = RPM * 2 * Math.PI / 60;
        dashboardTelemetry.addData("target RPM", MotorRPM);
        dashboardTelemetry.addData("current RPM", RPM);
        dashboardTelemetry.addData("target velocity (ticks/s)", TargetTicksPerSecond);
        dashboardTelemetry.addData("current velocity (ticks/s)", velocity);
        dashboardTelemetry.addData("tangential velocity (m/s)", velocityRad * wheelRadius);
        dashboardTelemetry.addData("shooter power", shooter.getPower());

        String currentDraw = hub.getFormattedCurrentDraw();
        dashboardTelemetry.addLine(currentDraw);

        dashboardTelemetry.update();
    }

    @Override
    public void stop() {
        shooter.setPower(0);
        controllers.stop();
    }
}
