package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.auto.FieldConstants;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;
import org.firstinspires.ftc.teamcode.robot.systems.BumperController;
import org.firstinspires.ftc.teamcode.robot.systems.HubController;
import org.firstinspires.ftc.teamcode.robot.systems.ShooterController;

@TeleOp(name="ShooterTesting", group="Iterative Opmode")
public class ShooterTesting extends OpMode {

    public static volatile double velocity;

    ShooterController shooter;
    ControllerManager controllers;
    BumperController bumper;
    HubController hub;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void init() {
        shooter = new ShooterController(hardwareMap, telemetry);
        controllers = new ControllerManager(telemetry);
        hub = new HubController(hardwareMap, telemetry);
        bumper = new BumperController(hardwareMap, telemetry);

        controllers.add(FieldConstants.Shooter, shooter);
        controllers.add(FieldConstants.Hub, hub);
        controllers.add("bumper", bumper);

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
        shooter.shootWithVelocity(velocity);
        telemetry.addData("velocity", shooter.getVelocity());
        dashboardTelemetry.addData("velocity", shooter.getVelocity());
        telemetry.addLine(hub.getFormattedCurrentDraw());
        dashboardTelemetry.addLine(hub.getFormattedCurrentDraw());

        if (gamepad1.a) bumper.bump();
        if (gamepad1.b) bumper.retract();
    }

    @Override
    public void stop() {
        controllers.stop();
    }
}
