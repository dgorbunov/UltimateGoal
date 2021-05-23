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
import static org.firstinspires.ftc.teamcode.util.Sleep.sleep;

@Config
public class ShooterController implements Controller {

    public static volatile double RetractDelay = 100;
    public static volatile double[] ShootingDelay = {250,250,0};
    public static volatile boolean useDelayArray = false;
    public static volatile double Delay1 = 1000;
    public static volatile double Delay2 = 1000;

    public static double BumperExtendPosition = 0.18;
    public static double BumperRetractPosition = 0.355;
    public static double TapperExtendPosition = 0.35;
    public static double TapperRetractPosition = 0.8;
    public static double TapperDelay = 300;

    private final double TICKS_PER_REV = 28.0; //Do not modify

    public static double MAX_VEL;
    public static volatile double kP = 6;
    public static volatile double kI = 0.2; //us``ed to rev up
    public static volatile double kD = 6.5;
    public static volatile double F = 11.7;
    public static volatile double kP2 = 5; //used for shooting
    public static volatile double kI2 = 0.3;
    public static volatile double kD2 = 0;
    public static double SHOOTER_ADMISSIBLE_ERROR = 75; //in RPM

    public volatile boolean shootingState;
    private volatile double targetRPM;
    private boolean stopWheelOnFinish = true;
    private static volatile boolean limitHit;
    private static volatile boolean lockTurret = false;

    public static double TURRET_CENTER_POS = 0.41;
    public static double TURRET_OFFSET = 3.5;
    public static double TURRET_MAX_ANGLE = 140;
    public static double TURRET_RIGHT_LIMIT = -29;
    public static double TURRET_LEFT_LIMIT = 75;
    public static int TURRET_UPDATE_RATE = 15;

    public static double SPEED_FUNC_SLOPE = 5.10833;
    public static double SPEED_FUNC_INTERCEPT = 2842.5;
    public static double SPEED_FUNC_POWERSHOT_OFFSET = -224;
    public static final double ANGLE_FUNC_SLOPE = 5.20833; //to account for shot curving
    public static final double ANGLE_FUNC_INTERCEPT = 2877.5;
    
    public static enum ShootingMode {
        GOAL, POWERSHOT
    };
    public static volatile ShootingMode shootingMode = ShootingMode.GOAL;

    public static volatile Pose2d robotPos = new Pose2d(FieldConstants.RedRight.StartingPos, 0);
    public static volatile Vector2d targetPos = GoalPos;
    private TurretThread turretThread = new TurretThread();

    public static DcMotorSimple.Direction Direction = DcMotorSimple.Direction.REVERSE;
    public static String ControllerName;

    private DcMotorEx shooter;
    private Servo bumper;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private MultipleTelemetry telemetryd;
    private Servo turret;
    private Servo tapper;

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
        tapper = hardwareMap.get(Servo.class, "tapper");
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(Direction);
        turretThread.start();

        shooter.setVelocityPIDFCoefficients(kP, kI, kD, F);

        //this is acting as our state variable
        TURRET_OFFSET = 3.5;
        shooter.setMotorDisable();
        bumperRetract();
    }

    @Override
    public void start() {
        shooter.setVelocityPIDFCoefficients(kP2, kI2, kD2, F);
        tapperRetract();
    }

    @Override
    public void stop() {
        stopShooter();
        turretThread.killThread();
        turret.setPosition(TURRET_CENTER_POS);
        tapperRetract();
        bumperExtend(); //to zero out position drift
    }

    public void stopShooter() {
        targetRPM = 0;
        shooter.setPower(0);
        shooter.setMotorDisable();
        bumper.setPosition(BumperRetractPosition);
        shootingState = false;
    }

    private void tapperRetract() {
        tapper.setPosition(TapperRetractPosition);
    }

    public void tapRings() {
        tapper.setPosition(TapperExtendPosition);
        Sleep.sleep(TapperDelay);
        tapper.setPosition(TapperRetractPosition);
    }

    public void updateTurret(Pose2d robotPos, Vector2d targetPos) {
        lockTurret = false;
        this.robotPos = robotPos;
        this.targetPos = targetPos;
        setRPM(calculateRPM(robotPos, targetPos, shootingMode));
    }

    public void updateTurretAuto(Pose2d robotPos, Vector2d targetPos) {
        lockTurret = false;
        this.robotPos = robotPos;
        this.targetPos = targetPos;
    }

    public void lockTurret(){
        lockTurret = true;
    }

    public void unlockTurret(){
        lockTurret = false;
    }

    public void turnTurret(Pose2d robotPos, Vector2d targetPos) {
        lockTurret = false;
        this.robotPos = robotPos;
        this.targetPos = targetPos;

        //Roadrunner uses inverted x/y axes
        double dX = targetPos.getY() - robotPos.getY();
        double dY = targetPos.getX() - robotPos.getX();
        double deg = -Math.toDegrees(Angle.normDelta(robotPos.getHeading())) + Math.toDegrees(Math.atan(dX/dY)) - TURRET_OFFSET;
//                    double deg = -Math.toDegrees(Angle.normDelta(robotPos.getHeading())); //for testing
        turret.setPosition(turnTurretAbsolute(deg));
    }

    private double calculateRPM(Pose2d robotPos, Vector2d goalPos, ShootingMode mode) {
        double a = goalPos.getY() - robotPos.getY();
        double b = goalPos.getX() - robotPos.getX();
        double distance = Math.sqrt(a*a + b*b) + 9; //measurement bias
        double speed = SPEED_FUNC_SLOPE * distance + SPEED_FUNC_INTERCEPT;

        if (mode == ShootingMode.POWERSHOT) return speed + SPEED_FUNC_POWERSHOT_OFFSET;
        return speed;
    }

    private double turnTurretAbsolute(double deg) {
        lockTurret = false;
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
                    if (!limitSpeak && limitHit) {
                        limitSpeak = true;
                        telemetry.speak("limit");
                    } else if (limitSpeak && !limitHit) limitSpeak = false;

                    sleep(TURRET_UPDATE_RATE);
                    if (!lockTurret) turnTurret(robotPos, targetPos);
                    else turret.setPosition(TURRET_CENTER_POS);
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
        shootingMode = ShootingMode.GOAL;
        if (!limitHit) new ShooterThread(ringCount, RPM).start();
    }

    public void shoot(int ringCount, double RPM, boolean stop){
        shootingState = true;
        stopWheelOnFinish = stop;
        shootingMode = ShootingMode.GOAL;

        setRPM(RPM);
        if (!limitHit) bumpRings(ringCount);
    }
    public void shootCheckSpeed(int ringCount, double RPM, boolean stop){
        shootingState = true;
        stopWheelOnFinish = stop;
        shootingMode = ShootingMode.GOAL;

        setRPM(RPM);
        if (!limitHit) bumpRingsCheckSpeed(RPM, ringCount);
    }

    public void bump(int ringCount){
        bumpRings(ringCount);
    }

    public synchronized void powerShot(double RPM){
        stopWheelOnFinish = false;
        shootingMode = ShootingMode.POWERSHOT;

        bumpRings(1);
        Sleep.sleep(150);
    }

    public synchronized void powerShotStrafe(double RPM){
        stopWheelOnFinish = false;
        shootingMode = ShootingMode.POWERSHOT;

        bumpRings(1);
    }

    private synchronized void checkSpeed(double targetRPM) {
        this.targetRPM = targetRPM;
        double ADMISSIBLE_ERROR = RPMtoTPS(SHOOTER_ADMISSIBLE_ERROR);
        double targetTPS = RPMtoTPS(targetRPM);

        NanoClock systemClock = NanoClock.system();
        double initialTime = systemClock.seconds();
        double maxDelay = 3; //while loop exits after maxDelay seconds
        int i = 0;

        RobotLog.clearGlobalWarningMsg();

        while (systemClock.seconds() - initialTime < maxDelay && (Math.abs(shooter.getVelocity() - targetTPS) > ADMISSIBLE_ERROR)){
            if (i == 0) telemetryd.addLine("Waiting for shooter to spin up");
            i++;
            sleep(15);
        }

       if (systemClock.seconds() - initialTime > maxDelay) {
           RobotLog.addGlobalWarningMessage("Shooter was unable to reach set velocity in " + maxDelay + " s");
       }
    }

    public String getSpeedStability() {
        double ADMISSIBLE_ERROR = RPMtoTPS(SHOOTER_ADMISSIBLE_ERROR);
        double targetTPS = RPMtoTPS(targetRPM);
        if ((Math.abs(shooter.getVelocity() - targetTPS) > ADMISSIBLE_ERROR)) return "<strong>[WARN] Shooter velocity is NOT stable</strong>";
        else return "<strong>Shooter velocity is stable</strong>";
    }

    public void incrementPower(double increment) {
        SPEED_FUNC_INTERCEPT += increment;
    }

    public void spinUp(double RPM){
        setRPM(RPM);
    }

    private void bumpRingsCheckSpeed(double RPM, int ringCount){
        //FTCDash doesn't support quick array modification
        //So we have to resort to this to tune delays =(
        if (useDelayArray) {
            for (int i = 0; i < ringCount; i++) {
                checkSpeed(RPM);
                bumperExtend();
                sleep(RetractDelay);
                bumperRetract();
//                sleep(ShootingDelay[i]);
            }
        } else if (ringCount == 3) {
            checkSpeed(RPM);
            bumperExtend();
            sleep(RetractDelay);
            bumperRetract();
            sleep(Delay1);
            checkSpeed(RPM);
            bumperExtend();
            sleep(RetractDelay);
            bumperRetract();
            sleep(Delay2);
            checkSpeed(RPM);
            bumperExtend();
            sleep(RetractDelay);
            bumperRetract();
        } else if (ringCount == 2){
            checkSpeed(RPM);
            bumperExtend();
            sleep(RetractDelay);
            bumperRetract();
           sleep(Delay1);
            checkSpeed(RPM);
            bumperExtend();
            sleep(RetractDelay);
            bumperRetract();
        } else {
            checkSpeed(RPM);
            bumperExtend();
            sleep(RetractDelay);
            bumperRetract();
        }

        shootingState = false;
        if (stopWheelOnFinish) stopShooter();
    }

    private void bumpRings(int ringCount){
        //FTCDash doesn't support quick array modification
        //So we have to resort to this to tune delays =(
        if (useDelayArray) {
            for (int i = 0; i < ringCount; i++) {
                bumperExtend();
                sleep(RetractDelay);
                bumperRetract();
                sleep(ShootingDelay[i]);
            }
        } else if (ringCount == 3) {
            bumperExtend();
            sleep(RetractDelay);
            bumperRetract();
            sleep(Delay1);
            bumperExtend();
            sleep(RetractDelay);
            bumperRetract();
            sleep(Delay2);
            bumperExtend();
            sleep(RetractDelay);
            bumperRetract();
        } else if (ringCount == 2){
            bumperExtend();
            sleep(RetractDelay);
            bumperRetract();
            sleep(Delay1);
            bumperExtend();
            sleep(RetractDelay);
            bumperRetract();
        } else {
            bumperExtend();
            sleep(RetractDelay);
            bumperRetract();
        }

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

    private void setRPM(double targetRPM) {
        this.targetRPM = targetRPM;
        shooter.setMotorEnable();
        shooter.setVelocity(RPMtoTPS(targetRPM));
    }

    private double RPMtoTPS(double RPM){
        return RPM * TICKS_PER_REV / 60.0;
    }

    private void bumperExtend() {
        bumper.setPosition(BumperExtendPosition);
    }
    private void bumperRetract() { bumper.setPosition(BumperRetractPosition); }

}