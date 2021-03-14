package org.firstinspires.ftc.teamcode.robot.systems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Controller;

import static org.firstinspires.ftc.teamcode.util.Sleep.sleep;

@Config
public class ShooterController implements Controller {

    public static volatile double SpinUpDelay = 1500;
    public static volatile double RetractDelay = 200;
    public static volatile double[] ShootingDelay = {250,250,0};
    public static volatile boolean useDelayArray = false;
    public static volatile double Delay1 = 300;
    public static volatile double Delay2 = 300;

    public static final double BumpPosition = 0.6;
    public static final double RetractPosition = 0.35;

    public volatile boolean shootingState;
    private boolean stopWheelOnFinish = true;
    private int powerShotCount = 0;

    public static DcMotorSimple.Direction Direction = DcMotorSimple.Direction.REVERSE;

    //Do not modify
    private final double TicksPerRev = 28;
    private final double wheelRadius = 0.051; //meters

    public static String ControllerName;

    private DcMotorEx shooter;
    private Servo bumper;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public ShooterController (HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        ControllerName = getClass().getSimpleName();
    }

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        bumper = hardwareMap.get(Servo.class, "bumper");

        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(Direction);

        //this is acting as our state variable
        shooter.setMotorDisable();

        retract();
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
        //wait until shooting is finished
        shooter.setPower(0);
        shooter.setMotorDisable();
        bumper.setPosition(RetractPosition);
        shootingState = false;
    }

    class ShooterThread extends Thread {
        double RPM;
        int ringCount;

        ShooterThread(int ringCount, double RPM) {
            this.ringCount = ringCount;
            this.RPM = RPM;
        }

        public void killThread() {
            interrupt();
        }

        public void run() {
            checkSpeed(RPM);
            bumpRings(ringCount);
            killThread();
        }
    }

    public synchronized void shootAsync(int ringCount, double RPM){
        shootingState = true;
        stopWheelOnFinish = true;
        new ShooterThread(ringCount, RPM).start();
    }

    public void shoot(int ringCount, double RPM){
        shootingState = true;
        stopWheelOnFinish = true;

        checkSpeed(RPM);
        bumpRings(ringCount);
    }


    public synchronized void powerShot(double RPM){
        stopWheelOnFinish = false;

        if (powerShotCount == 0 || powerShotCount == 3) {
            checkSpeed(RPM);
            powerShotCount = 0;
        }
        sleep(50); //buffer
        bumpRings(1);
        powerShotCount++;
    }

    private synchronized void checkSpeed(double RPM) {
        setRPM(RPM);

        NanoClock systemClock = NanoClock.system();
        double initialTime = systemClock.seconds();
        double maxDelay = 4; //while loop exits after maxDelay seconds
        int i = 0;

        while (systemClock.seconds() - initialTime < maxDelay && (shooter.getVelocity() < 0.975 *  TicksPerSecond(RPM) || shooter.getVelocity() > 1.025 *TicksPerSecond(RPM))){
            if (i == 0) telemetry.addLine("Waiting for shooter to spin up");
            i++;
        }

       if (systemClock.seconds() - initialTime > maxDelay) {
           RobotLog.clearGlobalWarningMsg();
           RobotLog.addGlobalWarningMessage("Shooter was unable to reach set velocity in " + maxDelay + " s");
       }

       sleep(100);
    }

    /**
     * Spin up flywheel before shooting to save time
     **/
    public void spinUp(double RPM){
        setRPM(RPM);
    }

    private void bumpRings(int ringCount){
        //FTCDash doesn't support quick array modification
        //So we have to resort to this to tune delays =(
        if (useDelayArray) {
            for (int i = 0; i < ringCount; i++) {
                bump();
                sleep(RetractDelay);
                retract();
                sleep(ShootingDelay[i]);
            }
        } else if (ringCount == 3) {
            bump();
            sleep(RetractDelay);
            retract();
            sleep(Delay1);
            bump();
            sleep(RetractDelay);
            retract();
            sleep(Delay2);
            bump();
            sleep(RetractDelay);
            retract();
        } else if (ringCount == 2){
            bump();
            sleep(RetractDelay);
            retract();
            sleep(Delay1);
            bump();
            sleep(RetractDelay);
            retract();
        } else {
            bump();
            sleep(RetractDelay);
            retract();
        }

        IntakeController.numRings.set(IntakeController.numRings.get() - ringCount);

        if (stopWheelOnFinish) stop();
    }

    public synchronized void telemetry(){
//        while (shootingState) {
//            double velocity = shooter.getVelocity();
//            double RPM = velocity / TicksPerRev * 60;
//            double velocityRad = RPM * 2 * Math.PI / 60;
//            dashboardTelemetry.addData("target RPM", MotorRPM);
//            dashboardTelemetry.addData("current RPM", RPM);
//            dashboardTelemetry.addData("tangential velocity (m/s)", velocityRad * wheelRadius);
//            dashboardTelemetry.addData("shooter power", shooter.getPower());
//
//            dashboardTelemetry.update();
//        }
//
//        dashboardTelemetry.addData("shooter stopped", shooter.getPower());
//        dashboardTelemetry.update();
    }

    private void setRPM(double RPM) {
        shooter.setMotorEnable();
        shooter.setVelocity(TicksPerSecond(RPM));
    }

    private double TicksPerSecond(double RPM){
        return RPM * TicksPerRev / 60;
    }

    private void bump() {
        bumper.setPosition(BumpPosition);
    }
    private void retract() {
        bumper.setPosition(RetractPosition);
    }

}