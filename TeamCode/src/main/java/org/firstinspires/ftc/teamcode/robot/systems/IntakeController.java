package org.firstinspires.ftc.teamcode.robot.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.Controller;
import org.firstinspires.ftc.teamcode.util.Sleep;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;

@Config
public class IntakeController implements Controller {
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private DcMotorEx intake;
    private DcMotorEx intake2;
    private Servo arm;
    private static CRServo sweeper;
    private DistanceSensor intakeSensor;

    public static String ControllerName;
    public static boolean intakeRunning;
    public volatile AtomicLong lastSensorReading = new AtomicLong(0);
    private SensorThread sensorThread = new SensorThread(DistanceUnit.CM);
    public static double SensorThreshold = 21.0;
    public static volatile AtomicInteger numRings = new AtomicInteger(0);

    public static double ArmStartPos = 0.1;
    public static double ArmDropPos = 0.35;
    public static double IntakePower = 0.7;
    public static double Intake2Power = 0.7;
    public static double SweeperPower = 1.;
    public static double sensorMaxDistance = 2;

    /*
    * Do not change motor direction to avoid breaking odometry
    * which uses the same encoder ports
    * */

    public static Servo.Direction ArmDirection = Servo.Direction.REVERSE;
    public static CRServo.Direction SweeperDirection = CRServo.Direction.REVERSE;

    public IntakeController(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        ControllerName = getClass().getSimpleName();
    }

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake2 = hardwareMap.get(DcMotorEx.class, "intake2");
        arm = hardwareMap.get(Servo.class, "intake_arm");
        sweeper = hardwareMap.get(CRServo.class, "sweeper");
        intakeSensor = hardwareMap.get(DistanceSensor.class, "intake_sensor");

        arm.setDirection(ArmDirection);
        sweeper.setDirection(SweeperDirection);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setPosition(ArmStartPos);
        numRings.set(0);
    }

    @Override
    public void start() {

    }

    public void extend() {
        arm.setPosition(ArmDropPos);
    }
    public void retract() { arm.setPosition(ArmStartPos); }

    @Override
    public void stop() {
        stopIntake();
        arm.setPosition(ArmStartPos);
    }

    public void stopIntake() {
        intakeRunning = false;
        intake.setPower(0);
        intake2.setPower(0);
        sensorThread.stopThread();
        stopSweeper();
    }

    public void stopIntake(boolean stopSweeper) {
        intakeRunning = false;
        intake.setPower(0);
        intake2.setPower(0);
        sensorThread.stopThread();
        if (stopSweeper) stopSweeper();
    }

    public static void startSweeper() {
        sweeper.setPower(SweeperPower);
    }

    private static void startSweeper(double power) {
        if (!VertIntakeController.isRunning) sweeper.setPower(power);
    }

    public static void stopSweeper() {
        if (!intakeRunning && !VertIntakeController.isRunning) sweeper.setPower(0);
    }

    public static void resetRingCount() { numRings.set(0); }

    public void runAuto(DcMotorEx.Direction Direction) {
        new AutomaticIntakeThread(Direction).start();
    }

    public double getSensorDistance() {
        return lastSensorReading.doubleValue();
    }

    public void run(DcMotorEx.Direction Direction) {
        intakeRunning = true;
        if (!sensorThread.isAlive()) sensorThread.start();
        telemetry.addData(ControllerName, "Intaking");

        if (Direction == DcMotorSimple.Direction.FORWARD) {
            intake.setPower(IntakePower);
            intake2.setPower(Intake2Power);
            startSweeper(SweeperPower);
        }
        else {
            intake.setPower(-IntakePower);
            intake2.setPower(-Intake2Power);
            startSweeper(-SweeperPower);
        }
    }

    class SensorThread extends Thread {
        public long SENSOR_POLLING_RATE = 250;
        private DistanceUnit unit;
        private AtomicBoolean isRunning = new AtomicBoolean(false);
        boolean ringState;

        SensorThread(DistanceUnit unit) {
            this.unit = unit;
        }

        public void run() {
            isRunning.set(true);
            while (isRunning.get()) {
                try {
                    sleep(SENSOR_POLLING_RATE);
                    lastSensorReading.set((long)intakeSensor.getDistance(unit));
                    if (!ringState && lastSensorReading.get() < SensorThreshold) {
                        ringState = true;
                        numRings.getAndIncrement();
                    } else if (ringState && lastSensorReading.get() >= SensorThreshold) ringState = false;
                } catch (InterruptedException e) {
                    lastSensorReading.set(0);
                    //TODO: sensor reading limited to ints
                    interrupt();
                    e.printStackTrace();
                    RobotLog.addGlobalWarningMessage("Intake sensor prematurely interrupted");
                }
            }
        }
        public void stopThread() {
            isRunning.set(false);
            lastSensorReading.set(0);
        }
    }

    class AutomaticIntakeThread extends Thread {
        DcMotorSimple.Direction Direction;
        int ringCount;
        boolean runIntake = true;
        boolean ringDetected = false;

        AutomaticIntakeThread(DcMotorEx.Direction Direction) {
            this.Direction = Direction;
        }

        public void run() {
            if (runIntake) {
                IntakeController.this.run(Direction);
                runIntake = false;
            }

            double reading = intakeSensor.getDistance(DistanceUnit.INCH);

            if (!ringDetected && reading <= sensorMaxDistance && reading >= 0) {
                ringCount++;
                ringDetected = true;
                if (ringCount >= 3) stopThread();
            }
            else if (reading >= sensorMaxDistance && reading >= 0) ringDetected = false;

            Sleep.sleep(15);
        }

        public void stopThread() {
            stopIntake();
            interrupt();
        }
    }
}
