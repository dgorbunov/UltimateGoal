package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;
import org.firstinspires.ftc.teamcode.robot.systems.HubController;

import static org.firstinspires.ftc.teamcode.util.Sleep.sleep;

@Config
@TeleOp(name="ShooterTesting", group="Iterative Opmode")
public class ShooterTesting extends OpMode {

    /** Accessible via FTC Dashboard */
    public static volatile double MotorRPM = 4800;
    //public static volatile double IntakeRPM = 1400;
    public static volatile double IntakePower = 0.8;
    public static volatile double ShootingDelay = 750;
    public static volatile double BumpPosition = 0.6;
    public static volatile double RetractPosition = 0.4;
    
    public final double TicksPerRev = 28; //1:1 5202
    public final double IntakeTicksPerRev = 25.9; //3.7 NeveRest
    
    public static volatile double TargetTicksPerSecond; //x rev/min * 2pi = x rad/min / 60 = x rad/sec;;
    public static volatile double TargetTicksPerSecondIntake; //x rev/min * 2pi = x rad/min / 60 = x rad/sec;;
    float wheelRadius = 0.051f; //meters

    DcMotorEx shooter;
    DcMotorEx intake;
    Servo bumper;
    ControllerManager controllers;
    HubController hub;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    Thread shootThread = new Thread(this::shoot);
    Thread telemetryThread = new Thread(this::telemetry);
    Thread motorThread = new Thread(this::runMotor);
    Thread intakeThread = new Thread(this::runIntake);

    @Override
    public void init(){
        dashboardTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

        dashboardTelemetry.addLine("Initializing...");
        bumper = hardwareMap.get(Servo.class, "bumper");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

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
//        TargetTicksPerSecondIntake = IntakeRPM * IntakeTicksPerRev / 60;

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

        intakeThread.run();
    }

    private synchronized void shoot(){
        for (int i = 0; i < 3; i++) {
            bumper.setPosition(BumpPosition);
            bumper.setPosition(RetractPosition);
            sleep(ShootingDelay);
        }
    }

    private synchronized void runMotor() {
        dashboardTelemetry.addLine("Shooter is running");
        shooter.setVelocity(TargetTicksPerSecond);
    }

    private synchronized void runIntake(){
        dashboardTelemetry.addLine("Intake is running");
//        intake.setVelocity(TargetTicksPerSecondIntake);
        intake.setPower(IntakePower);
    }

    private synchronized void telemetry(){
        double velocity = shooter.getVelocity();
        double RPM = velocity / TicksPerRev * 60;
        double velocityRad = RPM * 2 * Math.PI / 60;
        dashboardTelemetry.addData("shooter target RPM", MotorRPM);
        dashboardTelemetry.addData("shooter current RPM", RPM);
        dashboardTelemetry.addData("shooter target velocity (ticks/s)", TargetTicksPerSecond);
        dashboardTelemetry.addData("shooter current velocity (ticks/s)", velocity);
        dashboardTelemetry.addData("shooter tangential velocity (m/s)", velocityRad * wheelRadius);
        dashboardTelemetry.addData("shooter power", shooter.getPower());

//        double intakeVelocity = intake.getVelocity();
//        double intakeRPM = intakeVelocity / IntakeTicksPerRev * 60;
//        dashboardTelemetry.addData("intake target RPM", IntakeRPM);
//        dashboardTelemetry.addData("intake current RPM", intakeRPM);

        String currentDraw = hub.getFormattedCurrentDraw();
        dashboardTelemetry.addLine(currentDraw);

        dashboardTelemetry.update();
    }

    @Override
    public void stop() {
        shooter.setPower(0);
        intake.setPower(0);
        controllers.stop();
    }
}
