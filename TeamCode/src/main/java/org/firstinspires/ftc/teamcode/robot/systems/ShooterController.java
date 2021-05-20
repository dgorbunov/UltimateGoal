package org.firstinspires.ftc.teamcode.robot.systems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;
import org.firstinspires.ftc.teamcode.robot.Controller;
import org.firstinspires.ftc.teamcode.util.Sleep;

import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField.GoalPos;
import static org.firstinspires.ftc.teamcode.robot.systems.IntakeController.numRings;
import static org.firstinspires.ftc.teamcode.util.Sleep.sleep;

@Config
public class ShooterController implements Controller {

    public static volatile double RetractDelay = 200;
    public static volatile double[] ShootingDelay = {250,250,0};
    public static volatile boolean useDelayArray = false;
    public static volatile double Delay1 = 750;
    public static volatile double Delay2 = 750;

    public static double ServoBumpPosition = 0.18;
    public static double ServoRetractPosition = 0.33;

    private final double TICKS_PER_REV = 28; //Do not modify

    public static double MAX_VEL;
    public static double kP = 2;
    public static double kI = 0.2;
    public static double kD = 6.5;
    public static double F = 11.7;
    public static double SHOOTER_ADMISSIBLE_ERROR = 40; //RPM

    public volatile boolean shootingState;
    private volatile double targetRPM;
    private boolean stopWheelOnFinish = true;
    private int powerShotCount = 0;
    private static volatile boolean limitHit;

    public static double TURRET_CENTER_POS = 0.41;
    public static double TURRET_OFFSET = 6;
    public static double TURRET_MAX_ANGLE = 140;
    public static double TURRET_RIGHT_LIMIT = -17;
    public static double TURRET_LEFT_LIMIT = 75;
    public static int TURRET_UPDATE_RATE = 15;

    public static volatile Pose2d robotPos = new Pose2d(FieldConstants.RedRight.StartingPos, 0);
    public static volatile Vector2d targetPos = GoalPos;
    private TurretThread turretThread = new TurretThread();

    /**
     * Origin in center
     * ===========================
     * |               | 0 deg
     * |               |
     * |               |
     * |               |
     * |             _____
     * | -90 -------|  •  |---------- +90 deg
     */


    /**
     * Servo positions
     * ===========================
     * |               | centerPos
     * |               |
     * |               |
     * |               |
     * |             _____
     * | 1   -------|  •  |---------- 0
     */

    public static DcMotorSimple.Direction Direction = DcMotorSimple.Direction.REVERSE;
    public static String ControllerName;

    private DcMotorEx shooter;
    private Servo bumper;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private MultipleTelemetry telemetryd;
    private Servo turret;

    public ShooterController(HardwareMap hardwareMap, Telemetry telemetry, MultipleTelemetry telemetryd) {
        this.telemetry = telemetry;
        this.telemetryd = telemetryd;
        this.hardwareMap = hardwareMap;
        ControllerName = getClass().getSimpleName();
    }

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        bumper = hardwareMap.get(Servo.class, "bumper");
        turret = hardwareMap.get(Servo.class, "turret");
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(Direction);
        turretThread.start();

        shooter.setVelocityPIDFCoefficients(kP, kI, kD, F);
        //TODO: re-tune
        telemetry.addData("F", F);

        //this is acting as our state variable
        shooter.setMotorDisable();
        retract();
    }

    @Override
    public void start() { }

    @Override
    public void stop() {
        stopShooter();
        turretThread.killThread();
    }

    public void stopShooter() {
        targetRPM = 0;
        shooter.setPower(0);
        shooter.setMotorDisable();
        bumper.setPosition(ServoRetractPosition);
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

    public void updateTurret(Pose2d robotPos, Vector2d targetPos) {
        this.robotPos = robotPos;
        this.targetPos = targetPos;
    }

    public void turnTurret(Pose2d robotPos, Vector2d targetPos) {
        this.robotPos = robotPos;
        this.targetPos = targetPos;

        //Roadrunner uses inverted x/y axes
        double dX = targetPos.getY() - robotPos.getY();
        double dY = targetPos.getX() - robotPos.getX();
        double deg = -Math.toDegrees(Angle.normDelta(robotPos.getHeading())) + Math.toDegrees(Math.atan(dX/dY)) - TURRET_OFFSET;
//                    double deg = -Math.toDegrees(Angle.normDelta(robotPos.getHeading())); //for testing
        turret.setPosition(turnTurretAbsolute(deg));
    }

    private double turnTurretAbsolute(double deg) {
        double degree = deg;

        if (degree > TURRET_LEFT_LIMIT) {
            degree = TURRET_LEFT_LIMIT;
            limitHit = true;
        }
        else if (degree < TURRET_RIGHT_LIMIT) {
            degree = TURRET_RIGHT_LIMIT;
            limitHit = true;
        }
        else limitHit = false;
        return ((degree / TURRET_MAX_ANGLE) + TURRET_CENTER_POS);
    }

    public void setVelocityPIDFCoefficients(double kP, double kI, double kD, double F) {
        shooter.setVelocityPIDFCoefficients(kP, kI, kD, F);
    }

    @Config
    class TurretThread extends Thread {
        boolean isRunning = true;
        boolean limitSpeak;

        public void run() {
            turret.setPosition(TURRET_CENTER_POS);
            while (isRunning) {
                try {
                    sleep(TURRET_UPDATE_RATE);
                    turnTurret(robotPos, targetPos);

                    if (!limitSpeak && limitHit) {
                        limitSpeak = true;
                        telemetry.speak("limit");
                    } else if (limitSpeak && !limitHit) limitSpeak = false;
                }
                catch (InterruptedException e) {
                    e.printStackTrace();
                    RobotLog.addGlobalWarningMessage("turret interrupted");
                }

            }
            turret.setPosition(TURRET_CENTER_POS);
            interrupt();
        }

        public void killThread() {
            isRunning = false;
        }
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

    public synchronized void shootAsync(int ringCount, double RPM){
        shootingState = true;
        stopWheelOnFinish = true;
        new ShooterThread(ringCount, RPM).start();
    }

    public void shoot(int ringCount, double RPM, boolean stop){
        shootingState = true;
        stopWheelOnFinish = stop;

        setRPM(RPM);
        checkSpeed(RPM);
        bumpRings(ringCount);
    }

    public synchronized void powerShot(double RPM){
        stopWheelOnFinish = false;

        checkSpeed(RPM);
        bumpRings(1);
        powerShotCount++;
        Sleep.sleep(150);
    }

    public synchronized void powerShotStrafe(double RPM){
        stopWheelOnFinish = false;
        bumpRings(1);
        powerShotCount++;
    }

    private synchronized void checkSpeed(double targetRPM) {
        this.targetRPM = targetRPM;
        double ADMISSIBLE_ERROR = RPMtoTPS(SHOOTER_ADMISSIBLE_ERROR);
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

    public String getSpeedStability() {
        double ADMISSIBLE_ERROR = RPMtoTPS(SHOOTER_ADMISSIBLE_ERROR);
        double targetTPS = RPMtoTPS(targetRPM);
        if ((Math.abs(shooter.getVelocity() - targetTPS) > ADMISSIBLE_ERROR)) return "<strong>Waiting for shooter velocity to stabilize</strong>";
        else return "<strong>Shooter velocity is stable</strong>";
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

        shootingState = false;
        if (stopWheelOnFinish) stopShooter();
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

        shootingState = false;
        if (stopWheelOnFinish) stopShooter();
    }

    public void incrementTurretOffset(double inc) {
        TURRET_OFFSET += inc;
    }

    public double getCurrentRPM(){
        return shooter.getVelocity() / TICKS_PER_REV * 60;
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
        return RPM * TICKS_PER_REV / 60;
    }

    private void bump() {
        bumper.setPosition(ServoBumpPosition);
    }
    private void retract() {
        bumper.setPosition(ServoRetractPosition);
    }

}