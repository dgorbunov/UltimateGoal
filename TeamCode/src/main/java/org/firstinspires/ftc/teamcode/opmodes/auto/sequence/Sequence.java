package org.firstinspires.ftc.teamcode.opmodes.auto.sequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;
import org.firstinspires.ftc.teamcode.robot.drive.DriveLocalizationController;
import org.firstinspires.ftc.teamcode.robot.systems.IntakeController;
import org.firstinspires.ftc.teamcode.robot.systems.ShooterController;
import org.firstinspires.ftc.teamcode.robot.systems.WobbleController;

import static org.firstinspires.ftc.teamcode.util.Sleep.sleep;

public abstract class Sequence {

    protected int ringCount;
    protected Telemetry telemetry;
    protected ControllerManager controllers;
    protected final static Object lock = new Object();
    protected Actions actions;
    protected Pose2d startPose;
    protected Vector2d targetZone;

    //TODO: Build all trajectories before running?

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
            DriveLocalizationController drive = controllers.get(DriveLocalizationController.class, FieldConstants.Drive);
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
        DriveLocalizationController drive = controllers.get(DriveLocalizationController.class, FieldConstants.Drive);
        if (drive != null) {
            return drive.getPoseEstimate();
        }

        return startPose;
    }

    public void moveLinear(Vector2d posititon, double targetHeading){
        turn(targetHeading);
        followTrajectoryAsync(buildLineTrajectory(posititon));
    }

    public void moveToZone(Vector2d targetZone, Vector2d intermediatePos, double targetHeading) throws IllegalArgumentException {
        telemetry.addData("Sequence", "moveToZone");
        turn(targetHeading);
//        followTrajectoryAsync(buildSplineTrajectory(intermediatePos, initialHeading, targetHeading));
//        followTrajectoryAsync(buildSplineTrajectory(targetZone, initialHeading, targetHeading));

        Pose2d positions[] = new Pose2d[] {
                new Pose2d(intermediatePos, targetHeading),
                new Pose2d(targetZone, targetHeading),
        };

        followTrajectoryAsync(buildSplineTrajectory(positions));
    }

    public void dropWobble() {
        telemetry.addData("Sequence","dropWobble");
        WobbleController wobble = controllers.get(WobbleController.class, FieldConstants.Wobble);
        wobble.drop();
    }

    public void moveToStart(Vector2d wobblePos, double initialHeading, double targetHeading) {
        telemetry.addData("Sequence","moveToStart");
        turn(targetHeading);
        followTrajectoryAsync(buildSplineTrajectory(wobblePos, initialHeading, targetHeading));
    }

    public void pickupWobble() {
        telemetry.addData("Sequence", "collectWobble");
        WobbleController wobble = controllers.get(WobbleController.class, FieldConstants.Wobble);
        wobble.pickup();
    }

    public void moveToShoot(Vector2d position, double heading) {
        telemetry.addData("Sequence","moveToShoot" );
        followTrajectoryAsync(buildLineTrajectory(position));
    }

    public void startShooter(double RPM){
        ShooterController shooter = controllers.get(ShooterController.class, FieldConstants.Shooter);
        shooter.spinUp(RPM);
    }

    public void stopShooter(){
        ShooterController shooter = controllers.get(ShooterController.class, FieldConstants.Shooter);
        shooter.stop();
    }

    public void shootRings(int numRings) {
        telemetry.addData("Sequence","shootRings: " + numRings);
        ShooterController shooter = controllers.get(ShooterController.class, FieldConstants.Shooter);
        shooter.shoot(numRings);
    }

    public void intakeRings(int numRings, Vector2d position, double heading) {
        IntakeController intake = controllers.get(IntakeController.class, FieldConstants.Intake);
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
        DriveLocalizationController drive = controllers.get(DriveLocalizationController.class, FieldConstants.Drive);
        Trajectory trajectory = drive.trajectoryBuilder(GetCurrentPose())
                .splineTo(position, Math.toRadians(targetHeading))
                .build();

        return trajectory;
    }

    private Trajectory buildSplineTrajectory(Pose2d[] positions){
        DriveLocalizationController drive = controllers.get(DriveLocalizationController.class, FieldConstants.Drive);
        TrajectoryBuilder trajectoryBuilder = drive.trajectoryBuilder(GetCurrentPose());
        for (Pose2d position : positions) {
            trajectoryBuilder.splineTo(position.vec(), Math.toRadians(position.getHeading()));
        }

        Trajectory trajectory = trajectoryBuilder.build();
        return trajectory;
    }

    private Trajectory buildConstantSplineTrajectory(Vector2d position, double heading){
        DriveLocalizationController drive = controllers.get(DriveLocalizationController.class, FieldConstants.Drive);
        TrajectoryBuilder trajectory = drive.trajectoryBuilder(GetCurrentPose())
                .splineToConstantHeading(position, Math.toRadians(heading));

        return trajectory.build();
    }

    private Trajectory buildLineTrajectory(Vector2d position){
        DriveLocalizationController drive = controllers.get(DriveLocalizationController.class, FieldConstants.Drive);
        Trajectory trajectory = drive.trajectoryBuilder(GetCurrentPose())
                .lineTo(position)
                .build();
        return trajectory;
    }

    private Trajectory buildStrafeTrajectory(Vector2d position){
        DriveLocalizationController drive = controllers.get(DriveLocalizationController.class, FieldConstants.Drive);
        Trajectory trajectory = drive.trajectoryBuilder(GetCurrentPose())
                .strafeTo(position)
                .build();
        return trajectory;
    }

    private void followTrajectoryAsync(Trajectory trajectory){
        if (trajectory == null) {
            telemetry.addData("Sequence", "moveToZone: invalid trajectory");
            throw new IllegalArgumentException("Invalid trajectory");
        }

        DriveLocalizationController drive = controllers.get(DriveLocalizationController.class, FieldConstants.Drive);
        drive.followTrajectoryAsync(trajectory);
        drive.waitForIdle();
        //although this may look equivalent to followTrajectory it is non-blocking
        //because sequences run on a separate thread, drive methods do not
    }

    private Trajectory buildIntakeTrajectory(Vector2d position, double heading, double timeDelay){
        DriveLocalizationController drive = controllers.get(DriveLocalizationController.class, FieldConstants.Drive);
        IntakeController intake = controllers.get(IntakeController.class, FieldConstants.Intake);
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
        DriveLocalizationController drive = controllers.get(DriveLocalizationController.class, FieldConstants.Drive);
        drive.turnAsync(Math.toRadians(heading));
        drive.waitForIdle();
    }
}
