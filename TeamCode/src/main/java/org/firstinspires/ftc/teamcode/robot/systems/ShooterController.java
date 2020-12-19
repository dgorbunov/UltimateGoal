package org.firstinspires.ftc.teamcode.robot.systems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.opmodes.auto.FieldConstants;
import org.firstinspires.ftc.teamcode.robot.Controller;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;
import org.firstinspires.ftc.teamcode.util.MockDcMotorEx;

import static org.firstinspires.ftc.teamcode.util.Sleep.sleep;

public class ShooterController implements Controller {

    public static volatile double MotorRPM = 4800;
    public static volatile double ShootingDelay = 750;
    public static volatile double SpinUpDelay = 750;
    public static volatile double BumpPosition = 0.6;
    public static volatile double RetractPosition = 0.4;

    private final double TicksPerRev = 28; //Do not modify

    public static String ControllerName;
    public boolean shootingState = false;

    public static volatile double TargetTicksPerSecond; //x rev/min * 2pi = x rad/min / 60 = x rad/sec;;
    float wheelRadius = 0.051f; //meters

    private DcMotorEx shooter;
    private BumperController bumper;
    private ControllerManager controllers;
    private HardwareMap hardwareMap;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public Thread telemetryThread = new Thread(this::telemetry);
    public Thread motorThread = new Thread(this::setVelocity);

    public ShooterController (HardwareMap hardwareMap, Telemetry telemetry) {
        this.dashboardTelemetry = telemetry;
        this.hardwareMap = hardwareMap;
        ControllerName = getClass().getSimpleName();
        shooter = new MockDcMotorEx("shooter", this.dashboardTelemetry);
    }

    @Override
    public void init() {
        dashboardTelemetry = new MultipleTelemetry(dashboardTelemetry, dashboardTelemetry);

        controllers = new ControllerManager(dashboardTelemetry);
        bumper = new BumperController(hardwareMap, dashboardTelemetry);

        controllers.add(FieldConstants.Bumper, bumper);

        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        TargetTicksPerSecond = MotorRPM * TicksPerRev / 60;

        controllers.init();
    }

    @Override
    public void start() {
        controllers.start();
    }

    @Override
    public void stop() {
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setPower(0);
    }

    public void shoot(double RPM, int ringCount){
        shootingState = true;
        MotorRPM = RPM;

        motorThread.start();
        telemetryThread.start();
        sleep(SpinUpDelay);
        shootImpl(ringCount);

        shootingState = false;
    }

    private void shootImpl(int ringCount){
        for (int i = 0; i < ringCount; i++) {
            bumper.bump();
            bumper.retract();
            sleep(ShootingDelay);
        }
    }

    public synchronized void telemetry(){
        while (shootingState) {
            double velocity = shooter.getVelocity();
            double RPM = velocity / TicksPerRev * 60;
            double velocityRad = RPM * 2 * Math.PI / 60;
            dashboardTelemetry.addData("target RPM", MotorRPM);
            dashboardTelemetry.addData("current RPM", RPM);
            dashboardTelemetry.addData("tangential velocity (m/s)", velocityRad * wheelRadius);
            dashboardTelemetry.addData("shooter power", shooter.getPower());

            dashboardTelemetry.update();
        }
    }

    public synchronized void setVelocity() {
        while (shootingState) {
            dashboardTelemetry.addData(ControllerName, "shooter is running");
            TargetTicksPerSecond = MotorRPM * TicksPerRev / 60;
            shooter.setVelocity(TargetTicksPerSecond);
        }
    }

    public double getVelocity(){
        return shooter.getVelocity();
    }

}
