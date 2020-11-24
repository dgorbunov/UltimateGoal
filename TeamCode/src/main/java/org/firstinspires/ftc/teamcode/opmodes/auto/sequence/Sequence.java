package org.firstinspires.ftc.teamcode.opmodes.auto.sequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.auto.Constants;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;
import org.firstinspires.ftc.teamcode.robot.drive.DriveLocalizationController;
import org.firstinspires.ftc.teamcode.robot.systems.IntakeController;
import org.firstinspires.ftc.teamcode.robot.systems.ShooterController;
import org.firstinspires.ftc.teamcode.robot.systems.WobbleController;

public abstract class Sequence {

    protected int ringCount;
    protected Telemetry telemetry;
    protected ControllerManager controllers;
    protected final static Object theLock = new Object();
    protected Actions actions;
    protected Pose2d startPose;

    public Sequence(ControllerManager controllers, Telemetry tel) {
        this.telemetry = tel;
        this.controllers = controllers;
        this.actions = new Actions(tel);
    }

    public void init(int ringCount) {
        init(ringCount, startPose);
    }

    public void init(int ringCount, Pose2d startPose) {
        synchronized (theLock) {
            telemetry.addData("Sequence", "start pose: " + startPose.toString());
            this.ringCount = ringCount;
            this.startPose = startPose;

            // Reset all actions
            actions = new Actions(telemetry);
            makeActions();

            // Define our start pose
            DriveLocalizationController drive = (DriveLocalizationController)controllers.get(Constants.Drive);
            drive.setPoseEstimate(startPose);
        }
    }

    protected abstract void makeActions();

    public void execute() {
        telemetry.addData("Sequence", "Executing sequence on thread: " + Thread.currentThread().getId());

        // Do all the work on another thread to make this method not blocking
        // the main thread
        new Thread(() -> actions.run()).start();
    }

    public void stop() {
        telemetry.addData("Sequence", "stop");
        actions.stop();
    }

    public Pose2d GetCurrentPose() {
        DriveLocalizationController drive = (DriveLocalizationController)controllers.get(Constants.Drive);
        return drive.getPoseEstimate();
    }

    public void moveToZone(Vector2d targetZone, double heading) {
        telemetry.addData("Sequence", "moveToZone: " + targetZone.toString() + "," + heading);

        DriveLocalizationController drive = (DriveLocalizationController)controllers.get(Constants.Drive);
        Trajectory mySequence = drive.trajectoryBuilder(GetCurrentPose())
                .splineTo(targetZone, heading)
                .build();
        //TODO: Avoid rings

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
