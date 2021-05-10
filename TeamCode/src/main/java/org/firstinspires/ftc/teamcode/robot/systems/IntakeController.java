package org.firstinspires.ftc.teamcode.robot.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.Controller;

import java.util.concurrent.atomic.AtomicInteger;

@Config
public class IntakeController implements Controller {
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private DcMotorEx intake;
    private DcMotorEx intake2;
    private static CRServo topRoller;
    private static CRServo leftRoller;
    private static CRServo rightRoller;
    private DistanceSensor intakeSensor;

    public static String ControllerName;
    public static boolean isRunning;
    private SensorThread sensorThread = new SensorThread(DistanceUnit.CM);
    public static double SensorThreshold = 21.0;
    public static volatile AtomicInteger numRings = new AtomicInteger(0);
    public static int numRingsStale;

    public static double IntakePower = 0.8;
    public static double Intake2Power = 1.0;
    public static double RollerPower = 1.0;
    public static double HorizontalPower = 1.0;

    /*
    * Do not change motor direction to avoid breaking odometry
    * which uses the same encoder ports
    * */

    public static CRServo.Direction topRollerDirection = CRServo.Direction.REVERSE;
    public static CRServo.Direction leftRollerDirection = CRServo.Direction.FORWARD;
    public static CRServo.Direction rightRollerDirection = CRServo.Direction.FORWARD;

    public IntakeController(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        ControllerName = getClass().getSimpleName();
    }

    @Override
    public void init() {
        //TODO: fix swapped naming
        intake = hardwareMap.get(DcMotorEx.class, "intake2");
        intake2 = hardwareMap.get(DcMotorEx.class, "intake");
        topRoller = hardwareMap.get(CRServo.class, "top_roller");
        leftRoller = hardwareMap.get(CRServo.class, "left_roller");
        rightRoller = hardwareMap.get(CRServo.class, "right_roller");
//        intakeSensor = hardwareMap.get(DistanceSensor.class, "intake_sensor");

        topRoller.setDirection(topRollerDirection);
        leftRoller.setDirection(leftRollerDirection);
        rightRoller.setDirection(rightRollerDirection);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
        stopIntake();
    }

    public void stopIntake() {
        isRunning = false;
        intake.setPower(0);
        intake2.setPower(0);
        topRoller.setPower(0);
        leftRoller.setPower(0);
        rightRoller.setPower(0);
//        sensorThread.stopThread();
    }

    public void stopWheels() {
        intake2.setPower(0);
    }

    @Deprecated
    public void stopIntake(boolean stopSweeper) {
        stopIntake();
//        sensorThread.stopThread();
    }

    public static int getNumRings() {
        numRingsStale = numRings.get();
        return numRingsStale;
    }

//    public double getSensorReading() {
//        return sensorThread.lastSensorReading;
//    }

    public void runAuto(DcMotorEx.Direction Direction) {
        new AutomaticIntakeThread(Direction).start();
    }

    public void run(DcMotorEx.Direction Direction) {
        isRunning = true;
//        if (!sensorThread.isAlive()) sensorThread.start();
        telemetry.addData(ControllerName, "Intaking");

        if (Direction == DcMotorSimple.Direction.FORWARD) {
            intake.setPower(-IntakePower);
            intake2.setPower(-Intake2Power);
            topRoller.setPower(-RollerPower);
            leftRoller.setPower(-HorizontalPower);
            rightRoller.setPower(-HorizontalPower);
        }
        else {
            intake.setPower(IntakePower);
            intake2.setPower(Intake2Power);
            topRoller.setPower(RollerPower);
            leftRoller.setPower(HorizontalPower);
            rightRoller.setPower(HorizontalPower);
        }
    }

    public void runWheels(DcMotorEx.Direction Direction) {
        if (Direction == DcMotorSimple.Direction.FORWARD) {
            intake2.setPower(-Intake2Power);
        }
        else {
            intake2.setPower(Intake2Power);
        }
    }

    private void speak() {
        //TODO: crashing
        numRingsStale = numRings.get();
        telemetry.speak(numRingsStale + " Rings");
    }

    private class SensorThread extends Thread {
        public long SENSOR_POLLING_RATE = 50;
        private DistanceUnit unit;
        private volatile boolean isRunning = false;
        boolean ringState;
        public double lastSensorReading;

        SensorThread(DistanceUnit unit) {
            this.unit = unit;
        }

        public void run() {
            isRunning = true;
            while (isRunning) {
                try {
                    sleep(SENSOR_POLLING_RATE);
                    lastSensorReading = intakeSensor.getDistance(unit);
                    if (!ringState && lastSensorReading < SensorThreshold) {
                        ringState = true;
                        numRings.getAndIncrement();
                        telemetry.addData("Num Rings Intaked: ", numRings.get());
                    } else if (ringState && lastSensorReading >= SensorThreshold) ringState = false;
                } catch (InterruptedException e) {
                    lastSensorReading = 0;
                    interrupt();
                    e.printStackTrace();
                    RobotLog.addGlobalWarningMessage("Intake sensor prematurely interrupted");
                }
            }
            lastSensorReading = 0;
            interrupt();
        }
        public void stopThread() {
            isRunning = false;
        }
    }

    private class AutomaticIntakeThread extends Thread {
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

//            if (!ringDetected && reading <= sensorMaxDistance && reading >= 0) {
//                ringCount++;
//                ringDetected = true;
//                if (ringCount >= 3) stopThread();
//            }
//            else if (reading >= sensorMaxDistance && reading >= 0) ringDetected = false;
//
//            Sleep.sleep(15);
        }

        public void stopThread() {
            stopIntake();
            interrupt();
        }
    }
}
