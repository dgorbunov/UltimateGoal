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

import static org.firstinspires.ftc.teamcode.robot.systems.IntakeController.numRings;
import static org.firstinspires.ftc.teamcode.util.Sleep.sleep;

@Config
public class ShooterController implements Controller {

    public static volatile double RetractDelay = 200;
    public static volatile double[] ShootingDelay = {250,250,0};
    public static volatile boolean useDelayArray = false;
    public static volatile double Delay1 = 750;
    public static volatile double Delay2 = 750;

    public static final double BumpPosition = 0.6;
    public static final double RetractPosition = 0.35;

    private final double TicksPerRev = 28; //Do not modify
    private final double wheelRadius = 0.051; //meters

    public static double MAX_VEL;
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double F = 0;
    public static boolean useAutomaticPID = false;

    public volatile boolean shootingState;
    private volatile double targetRPM;
    private boolean stopWheelOnFinish = true;
    private int powerShotCount = 0;

    public static DcMotorSimple.Direction Direction = DcMotorSimple.Direction.REVERSE;

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

//        if (useAutomaticPID) setPID();
//        shooter.setVelocityPIDFCoefficients(kP, kI, kD, F);
        telemetry.addData("F", F);

        //this is acting as our state variable
        shooter.setMotorDisable();
        retract();
    }

    @Override
    public void start() { }

    @Override
    public void stop() {
        targetRPM = 0;
        shooter.setPower(0);
        shooter.setMotorDisable();
        bumper.setPosition(RetractPosition);
        shootingState = false;
    }

    private void setPID() {
        //https://docs.google.com/document/u/2/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/mobilebasic
        MAX_VEL = RPMtoTPS(6000);
        F = 32767 / MAX_VEL;
        kP = 0.1 * F;
        kI = 0.1 * kP;
        kD = 0;
    }

    public void setVelocityPIDFCoefficients(double kP, double kI, double kD, double F) {
        shooter.setVelocityPIDFCoefficients(kP, kI, kD, F);
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
            setRPM(RPM);
            checkSpeed(RPM);
            bumpRings(ringCount);
            killThread();
        }
    }

//    class TelemetryThread extends Thread {
//        double POLLING_RATE = 30;
//        private DrivetrainController drive;
//        boolean isAlive = true;
//
//        TelemetryThread(DrivetrainController drive) {
//            this.drive = drive;
//        }
//        //TODO: Do this properly eventually
//
//        public void run() {
//            while (isAlive)
//            Sleep.sleep(POLLING_RATE);
//            drive.putPacketData("shooter rpm", getCurrentRPM());
//            drive.update();
//        }
//
//        public void stopThread() {
//            isAlive = false;
//        }
//    }

    public synchronized void shootAsync(int ringCount, double RPM){
        shootingState = true;
        stopWheelOnFinish = true;
        new ShooterThread(ringCount, RPM).start();
    }

    public void shoot(int ringCount, double RPM){
        shootingState = true;
        stopWheelOnFinish = true;

        setRPM(RPM);
        checkSpeed(RPM);
        bumpRings(ringCount);
    }

//    public void shootWithTelemetry(DrivetrainController drive, int ringCount, double RPM){
//        shootingState = true;
//        stopWheelOnFinish = true;
//
//        TelemetryThread thread = new TelemetryThread(drive);
//        thread.start();
//        setRPM(RPM);
//        checkSpeed(RPM);
//        sleep(100);
//        bumpRings(ringCount);
//        thread.stopThread();
//    }

    public synchronized void powerShot(double RPM){
        stopWheelOnFinish = false;

//        if (powerShotCount == 0 || powerShotCount == 3) {
//            checkSpeed(RPM);
//            powerShotCount = 0;
//        }
        sleep(125); //buffer
        bumpRings(1);
        powerShotCount++;
    }

    public synchronized void powerShotStrafe(double RPM){
        stopWheelOnFinish = false;
        bumpRings(1);
        powerShotCount++;
    }

    private synchronized void checkSpeed(double RPM) {
        double ADMISSIBLE_ERROR = RPMtoTPS(30);
        double targetTPS = RPMtoTPS(targetRPM);

        NanoClock systemClock = NanoClock.system();
        double initialTime = systemClock.seconds();
        double maxDelay = 4.5; //while loop exits after maxDelay seconds
        int i = 0;

        while (systemClock.seconds() - initialTime < maxDelay && (Math.abs(shooter.getVelocity() - targetTPS) > ADMISSIBLE_ERROR)){
            if (i == 0) telemetry.addLine("Waiting for shooter to spin up");
            i++;
        }

       if (systemClock.seconds() - initialTime > maxDelay) {
           RobotLog.clearGlobalWarningMsg();
           RobotLog.addGlobalWarningMessage("Shooter was unable to reach set velocity in " + maxDelay + " s");
       }
    }

    /**
     * Spin up flywheel before shooting to save time
     **/
    public void spinUp(double RPM){
        setRPM(RPM);
    }

    private void bumpRingsCheckSpeed(double RPM, int ringCount){
        //FTCDash doesn't support quick array modification
        //So we have to resort to this to tune delays =(
        if (useDelayArray) {
            for (int i = 0; i < ringCount; i++) {
                checkSpeed(RPM);
                bump();
                sleep(RetractDelay);
                retract();
//                sleep(ShootingDelay[i]);
            }
        } else if (ringCount == 3) {
            checkSpeed(RPM);
            bump();
            sleep(RetractDelay);
            retract();
//            sleep(Delay1);
            checkSpeed(RPM);
            bump();
            sleep(RetractDelay);
            retract();
//            sleep(Delay2);
            checkSpeed(RPM);
            bump();
            sleep(RetractDelay);
            retract();
        } else if (ringCount == 2){
            checkSpeed(RPM);
            bump();
            sleep(RetractDelay);
            retract();
//            sleep(Delay1);
            checkSpeed(RPM);
            bump();
            sleep(RetractDelay);
            retract();
        } else {
            checkSpeed(RPM);
            bump();
            sleep(RetractDelay);
            retract();
        }

        numRings.set(numRings.get() - ringCount);
        if (numRings.get() < 0) numRings.set(0);

        if (stopWheelOnFinish) stop();
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

        numRings.set(numRings.get() - ringCount);
        if (numRings.get() < 0) numRings.set(0);

        if (stopWheelOnFinish) stop();
    }

    public double getCurrentRPM(){
        return shooter.getVelocity() / TicksPerRev * 60;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    private void setRPM(double RPM) {
        targetRPM = RPM;
        shooter.setMotorEnable();
        shooter.setVelocity(RPMtoTPS(targetRPM));
    }

    private double RPMtoTPS(double RPM){
        return RPM * TicksPerRev / 60;
    }

    private void bump() {
        bumper.setPosition(BumpPosition);
    }
    private void retract() {
        bumper.setPosition(RetractPosition);
    }

}