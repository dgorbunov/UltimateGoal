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
        telemetry.addData("Sequence", "Executing sequence on thread: " + Thread.currentThread().getId());

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
        telemetry.addData("Sequence", "moveToZone: " + targetZone.toString() + "," + heading);
        Trajectory mySequence = drive.trajectoryBuilder(GetCurrentPose())
                .splineTo(targetZone, heading)
                .build();

        drive.followTrajectory(mySequence);
    }

    // TODO: implement drop wobble
    public void dropWobble() {
        telemetry.addData("Sequence","dropWobble" );
        WobbleController wobble = (WobbleController)controllers.get(Constants.Wobble);
        wobble.start();
        wobble.drop();
    }

    // TODO: implement moveToStart
    public void moveToStart() {
        telemetry.addData("Sequence","moveToStart" );
    }

    // TODO: implement collect wobble
    public void collectWobble() {
        telemetry.addData("Sequence", "collectWobble" );
        WobbleController wobble = (WobbleController)controllers.get(Constants.Wobble);
        wobble.start();
        wobble.pickup();
    }

    // TODO: implement moveToShoot
    public void moveToShoot() {
        telemetry.addData("Sequence","moveToShoot" );
    }

    // TODO: implement the shooter
    public void shootRings() {
        telemetry.addData("Sequence","shootRings" );
        ShooterController shooter = (ShooterController)controllers.get(Constants.Shooter);
        shooter.start();
    }

    // TODO: implement the intake
    public void intakeRings() {
        telemetry.addData("Sequence","intakeRings" );
        IntakeController intake = (IntakeController)controllers.get(Constants.Intake);
        intake.start();
    }

    // TODO: implement moveToLaunchLine
    public void moveToLaunchLine() {
        telemetry.addData("Sequence","moveToLaunchLine" );
    }
}
