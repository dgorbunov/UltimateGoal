package org.firstinspires.ftc.teamcode.opmodes.auto.sequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.auto.Constants;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;
import org.firstinspires.ftc.teamcode.robot.camera.CameraController;
import org.firstinspires.ftc.teamcode.robot.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.systems.IntakeController;
import org.firstinspires.ftc.teamcode.robot.systems.ShooterController;
import org.firstinspires.ftc.teamcode.robot.systems.WobbleController;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

public abstract class Sequence {

    protected int ringCount;
    protected Telemetry telemetry;
    protected SampleMecanumDrive drive;
    protected ControllerManager controllers;
    protected final static Object theLock = new Object();
    protected Actions actions;

    public Sequence(ControllerManager controllers, HardwareMap hwMap, Telemetry tel) {
        this.drive = new SampleMecanumDrive(hwMap);
        this.telemetry = tel;
        this.controllers = controllers;
        this.actions = new Actions(tel);
    }

    public void init(int ringCount) {
        this.ringCount = ringCount;
        init(new Pose2d(0,0,0));
    }

    public void init(Pose2d startPose) {
        synchronized (theLock) {
            telemetry.addData("Sequence", "start pose: " + startPose.toString());

            // Reset all actions
            actions = new Actions(telemetry);

            // Define our start pose
            drive.setPoseEstimate(startPose);
        }
    }

    protected abstract void makeActions();

    public void execute() throws InterruptedException {
        telemetry.addData("Sequence", "nothing to execute");

        // Do all the work on another thread to make this method not blocking
        // the main thread
        new Thread(() -> actions.run()).start();
    }

    public void stop() {
        telemetry.addData("Sequence", "stop");
        drive.stop();
        actions.stop();
    }

    public Pose2d GetCurrentPose() {
        return drive.getPoseEstimate();
    }

    public void moveToZone(Vector2d targetZone, double heading) {
        pushTelemetry(); //TODO: This doesn't work?
        Trajectory mySequence = drive.trajectoryBuilder(GetCurrentPose())
                .splineTo(targetZone, heading)
                .build();

        drive.followTrajectory(mySequence);
    }

    // TODO: 11/21/2020 implement the trajectory execution
    public void dropWobble() {
        WobbleController wobble = (WobbleController)controllers.get(Constants.Wobble);
        wobble.start();
        wobble.drop();
    }

    // TODO: 11/21/2020 implement the trajectory execution
    public void moveToStart() {
    }

    // TODO: 11/21/2020 implement the trajectory execution
    public void collectWobble() {
        WobbleController wobble = (WobbleController)controllers.get(Constants.Wobble);
        wobble.start();
        wobble.pickup();
    }

    // TODO: 11/21/2020 implement the trajectory execution
    public void moveToShoot() {
    }

    // TODO: 11/21/2020 implement the shooter
    public void shootRings() {
        ShooterController shooter = (ShooterController)controllers.get(Constants.Shooter);
        shooter.start();
    }

    // TODO: 11/21/2020 implement the intake
    public void intakeRings() {
        IntakeController intake = (IntakeController)controllers.get(Constants.Intake);
        intake.start();
    }

    // TODO: 11/21/2020 implement the trajectory execution
    public void moveToLaunchLine() {
    }

    private void pushTelemetry() {
        StackTraceElement[] stacktrace = Thread.currentThread().getStackTrace();
        StackTraceElement e = stacktrace[2];//maybe this number needs to be corrected
        telemetry.addLine("Running: " + e.getMethodName());
    }
}
