package org.firstinspires.ftc.teamcode.opmodes.auto.sequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.auto.Constants;
import org.firstinspires.ftc.teamcode.opmodes.auto.FullAuto;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;
import org.firstinspires.ftc.teamcode.robot.drive.DriveLocalizationController;
import org.firstinspires.ftc.teamcode.robot.systems.IntakeController;
import org.firstinspires.ftc.teamcode.robot.systems.ShooterController;
import org.firstinspires.ftc.teamcode.robot.systems.WobbleController;

import java.util.LinkedList;
import java.util.Queue;

public abstract class Sequence {

    protected int ringCount;
    protected Telemetry telemetry;
    protected ControllerManager controllers;
    protected final static Object lock = new Object();
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
        synchronized (lock) {
            telemetry.addData("Sequence", "ringCount: " +ringCount + " start pose: " + startPose.toString());
            this.ringCount = ringCount;
            this.startPose = startPose;

            // Reset all actions
            actions = new Actions(telemetry);
            makeActions();

            // Define our start pose
            DriveLocalizationController drive = controllers.get(DriveLocalizationController.class, Constants.Drive);
            if (drive != null) {
                drive.setPoseEstimate(startPose);
            }
        }
    }

    protected abstract void makeActions();

    public void execute() {
        synchronized (lock) {
            telemetry.addData("Sequence", "Executing sequence on thread: " + Thread.currentThread().getId());

            // Do all the work on another thread to avoid blocking the invoking thread
            new Thread(() -> actions.run(FullAuto.TrajectorySelect)).start();
        }
    }

    public void stop() {
        synchronized (lock) {
            telemetry.addData("Sequence", "stop");
            actions.stop();
        }
    }

    public Pose2d GetCurrentPose() {
        DriveLocalizationController drive = controllers.get(DriveLocalizationController.class, Constants.Drive);
        if (drive != null) {
            return drive.getPoseEstimate();
        }

        return startPose;
    }

    public void moveToZone(Vector2d targetZone, double heading) {
        telemetry.addData("Sequence", "moveToZone: " + targetZone.toString() + "," + heading);

        DriveLocalizationController drive = controllers.get(DriveLocalizationController.class, Constants.Drive);

        followTrajectoryAsync(buildSplineTrajectory(targetZone, heading));
    }

    // TODO: implement drop wobble
    public void dropWobble() {
        telemetry.addData("Sequence","dropWobble" );
        WobbleController wobble = controllers.get(WobbleController.class, Constants.Wobble);
        wobble.drop();
    }

    // TODO: implement moveToStart
    public void moveToStart() {
        telemetry.addData("Sequence","moveToStart" );

    }

    // TODO: implement collect wobble
    public void collectWobble() {
        telemetry.addData("Sequence", "collectWobble" );
        WobbleController wobble = controllers.get(WobbleController.class, Constants.Wobble);
        if (wobble != null) {
            wobble.start();
            wobble.pickup();
        }
    }

    // TODO: implement moveToShoot
    public void moveToShoot() {
        telemetry.addData("Sequence","moveToShoot" );
    }

    // TODO: implement the shooter
    public void shootRings() {
        telemetry.addData("Sequence","shootRings" );
        ShooterController shooter = controllers.get(ShooterController.class, Constants.Shooter);
        if (shooter != null) {
            shooter.start();
        }
    }

    // TODO: implement the intake
    public void intakeRings() {
        telemetry.addData("Sequence","intakeRings" );
        IntakeController intake = controllers.get(IntakeController.class, Constants.Intake);
        if (intake != null) {
            intake.start();
        }
    }

    // TODO: implement moveToLaunchLine
    public void moveToLaunchLine() {
        telemetry.addData("Sequence","moveToLaunchLine" );
    }

    private Trajectory buildSplineTrajectory(Vector2d position, double heading){
        DriveLocalizationController drive = controllers.get(DriveLocalizationController.class, Constants.Drive);
        Trajectory trajectory = drive.trajectoryBuilder(GetCurrentPose())
                .splineTo(position, heading)
                .build();

        return trajectory;
    }

    private void followTrajectoryAsync(Trajectory trajectory){
        DriveLocalizationController drive = controllers.get(DriveLocalizationController.class, Constants.Drive);
        drive.followTrajectoryAsync(trajectory);
        drive.waitForIdle();
        //although this may look equivalent to followTrajectory it is non-blocking
        //because sequences run on a separate thread, drive methods do not
    }
}
