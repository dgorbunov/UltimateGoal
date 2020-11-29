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
import org.firstinspires.ftc.teamcode.util.Sleep;

import java.util.LinkedList;
import java.util.Queue;

import static org.firstinspires.ftc.teamcode.util.Sleep.sleep;

public abstract class Sequence {

    protected int ringCount;
    protected Telemetry telemetry;
    protected ControllerManager controllers;
    protected final static Object lock = new Object();
    protected Actions actions;
    protected Pose2d startPose;
    protected Vector2d targetZone;

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
            new Thread(() -> actions.run()).start();
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

    public void moveToZone(Vector2d targetZone, double initialHeading, double targetHeading) {
        telemetry.addData("Sequence", "moveToZone");
        turn(targetHeading);
        followTrajectoryAsync(buildSplineTrajectory(targetZone, initialHeading, targetHeading));
    }

    public void dropWobble() {
        telemetry.addData("Sequence","dropWobble");
        WobbleController wobble = controllers.get(WobbleController.class, Constants.Wobble);
        wobble.drop();
    }

    public void moveToStart(Vector2d targetZone, double initialHeading, double targetHeading) {
        telemetry.addData("Sequence","moveToStart");
        turn(targetHeading);
        followTrajectoryAsync(buildSplineTrajectory(targetZone, initialHeading, targetHeading));
    }

    public void pickupWobble() {
        telemetry.addData("Sequence", "collectWobble");
        WobbleController wobble = controllers.get(WobbleController.class, Constants.Wobble);
        wobble.pickup();
    }

    public void moveToShoot(Vector2d position, double heading) {
        telemetry.addData("Sequence","moveToShoot" );
        followTrajectoryAsync(buildConstantSplineTrajectory(position, heading));
    }

    public void shootRings() {
        telemetry.addData("Sequence","shootRings" );
        ShooterController shooter = controllers.get(ShooterController.class, Constants.Shooter);
        shooter.shoot(3000);
    }

    public void intakeRings(int numRings, Vector2d position, double heading) {
        IntakeController intake = controllers.get(IntakeController.class, Constants.Intake);
        switch (numRings) {
            case (0):
                telemetry.addData("Sequence", "no rings to intake");
                break;

            case (1):
                telemetry.addData("Sequence", "intake 1 ring");
                followTrajectoryAsync(buildIntakeTrajectory(position, heading, 0.5));
                sleep(1000);
                intake.stop();
                break;

            case (4):
                telemetry.addData("Sequence", "intake 4 rings");
                followTrajectoryAsync(buildIntakeTrajectory(position, heading, 0.5));
                sleep(3000);
                intake.stop();
                break;

        }
    }

    public void moveToLaunchLine(Vector2d position) {
        telemetry.addData("Sequence","moveToLaunchLine" );
        followTrajectoryAsync(buildLineTrajectory(position));
    }

    private Trajectory buildSplineTrajectory(Vector2d position, double initialHeading, double targetHeading){
        DriveLocalizationController drive = controllers.get(DriveLocalizationController.class, Constants.Drive);
        Trajectory trajectory = drive.trajectoryBuilder(GetCurrentPose())
//                .splineToLinearHeading(new Pose2d(position, Math.toRadians(targetHeading)), Math.toRadians(initialHeading))
                .splineTo(position, Math.toRadians(targetHeading))
                .build();

        return trajectory;
    }

    private Trajectory buildConstantSplineTrajectory(Vector2d position, double heading){
        DriveLocalizationController drive = controllers.get(DriveLocalizationController.class, Constants.Drive);
        Trajectory trajectory = drive.trajectoryBuilder(GetCurrentPose())
                .splineToConstantHeading(position, Math.toRadians(heading))
                .build();
        return trajectory;
    }

    private Trajectory buildLineTrajectory(Vector2d position){
        DriveLocalizationController drive = controllers.get(DriveLocalizationController.class, Constants.Drive);
        Trajectory trajectory = drive.trajectoryBuilder(GetCurrentPose())
                .lineTo(position)
                .build();
        return trajectory;
    }

    private Trajectory buildStrafeTrajectory(Vector2d position){
        DriveLocalizationController drive = controllers.get(DriveLocalizationController.class, Constants.Drive);
        Trajectory trajectory = drive.trajectoryBuilder(GetCurrentPose())
                .strafeTo(position)
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

    private Trajectory buildIntakeTrajectory(Vector2d position, double heading, double timeDelay){
        DriveLocalizationController drive = controllers.get(DriveLocalizationController.class, Constants.Drive);
        IntakeController intake = controllers.get(IntakeController.class, Constants.Intake);
        turn(heading);
        Trajectory trajectory = drive.trajectoryBuilder(GetCurrentPose())
                .lineTo(position)
                .addTemporalMarker(timeDelay, () -> { // This marker runs x # of seconds into the trajectory
                    intake.run(1);
                })
                .build();
        return trajectory;
    }

    private void turn(double heading){
        DriveLocalizationController drive = controllers.get(DriveLocalizationController.class, Constants.Drive);
        drive.turnAsync(Math.toRadians(heading));
        drive.waitForIdle();
    }
}
