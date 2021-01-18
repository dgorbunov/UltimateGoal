package org.firstinspires.ftc.teamcode.opmodes.auto.sequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;
import org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;
import org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController;
import org.firstinspires.ftc.teamcode.robot.drive.params.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.systems.IntakeController;
import org.firstinspires.ftc.teamcode.robot.systems.ShooterController;
import org.firstinspires.ftc.teamcode.robot.systems.WobbleController;

import java.util.Arrays;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
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

    public Sequence(ControllerManager controllers, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.controllers = controllers;
        this.actions = new Actions(this.telemetry);
    }

    public void init(int ringCount) {
        synchronized (lock) {
            telemetry.addData("Sequence", "ringCount: " +ringCount + " start pose: " + startPose.toString());
            this.ringCount = ringCount;

            // Reset all actions
            actions = new Actions(telemetry);
            makeActions();

            // Define our start pose
            DrivetrainController drive = controllers.get(DrivetrainController.class, FieldConstants.Drive);
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

            //Set starting position for TeleOp
            MechConstants.StartingPose = GetCurrentPose();
        }
    }

    public Pose2d GetCurrentPose() {
        try {
            DrivetrainController drive = controllers.get(DrivetrainController.class, FieldConstants.Drive);
            if (drive != null) {
                return drive.getPoseEstimate();
            }
        } catch (Exception e) {
            telemetry.addData("Sequence", "Could not fetch ending pose");
            telemetry.addData("Sequence", "Most likely stop pressed before start");
        }
        return startPose;
    }

    public void moveLinear(Vector2d posititon, double targetHeading) {
        followTrajectoryAsync(buildLinearTrajectory(posititon, targetHeading));
    }

    public void moveLinear(double x, double y, double targetHeading) {
        followTrajectoryAsync(buildLinearTrajectory(new Vector2d(x, y), targetHeading));
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
        wobble.dropAuto();
    }

    public void moveToWobble(Vector2d intermediate, Vector2d wobblePos, double targetHeading) {
        telemetry.addData("Sequence","moveToStart");
//        followTrajectoryAsync(buildSplineTrajectory(wobblePos, initialHeading, targetHeading));
        followTrajectoryAsync(buildBackTrajectory(18));
        followTrajectoryAsync(buildLinearTrajectory(intermediate, 0));
        followTrajectoryAsync(buildWobbleTrajectory(wobblePos, targetHeading));
    }

    public void pickupWobble() {
        telemetry.addData("Sequence", "collectWobble");
        WobbleController wobble = controllers.get(WobbleController.class, FieldConstants.Wobble);
        wobble.pickupAuto();
    }

    public void moveToShoot(Vector2d intermediate, Pose2d position) {
        telemetry.addData("Sequence","moveToShoot" );

        Pose2d positions[] = new Pose2d[] {
                new Pose2d(intermediate, 0),
                position
        };

//        followTrajectoryAsync(buildLineTrajectory(positions));
        followTrajectoryAsync(buildSplineTrajectory(positions));
        //TODO: fix, path continuity exception
    }

    public void moveToShoot(Pose2d position, double heading) {
        telemetry.addData("Sequence", "moveToShoot");
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

    public void shootGoal(int numRings) {
        telemetry.addData("Sequence","shootRings: " + numRings);
        ShooterController shooter = controllers.get(ShooterController.class, FieldConstants.Shooter);
        shooter.shootBlocking(numRings, MechConstants.RPMGoal);
    }

    public void backOffFromWobbles (double distance) {
        telemetry.addData("Sequence", "back off from wobbles");
        followTrajectoryAsync(buildBackTrajectory(distance));
    }

    public void shootPowershot(int numRings) {
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
                intake.extend();
                followTrajectoryAsync(buildIntakeTrajectory(position, heading, 0.25));
                sleep(3500);
                intake.stop();
                break;

            case (4):
                telemetry.addData("Sequence", "intake 4 rings");
                intake.extend();
                followTrajectoryAsync(buildIntakeTrajectory(position, heading, 0.25));
                sleep(7500);
                intake.stop();
                break;
            default:
                telemetry.addData("Sequence", "unsupported # of rings to intake");

        }
    }

    public void moveToLaunchLine(Vector2d position) {
        telemetry.addData("Sequence","moveToLaunchLine" );
        followTrajectoryAsync(buildLineTrajectory(new Pose2d(position, 0)));
    }

    private Trajectory buildSplineTrajectory(Vector2d position, double initialHeading, double targetHeading){
        DrivetrainController drive = controllers.get(DrivetrainController.class, FieldConstants.Drive);
        Trajectory trajectory = drive.trajectoryBuilder(GetCurrentPose())
                .splineTo(position, Math.toRadians(targetHeading))
                .build();

        return trajectory;
    }

    private Trajectory buildBackTrajectory(double inches) {
        DrivetrainController drive = controllers.get(DrivetrainController.class, FieldConstants.Drive);
        TrajectoryBuilder trajectory = drive.trajectoryBuilder(GetCurrentPose());
        trajectory.back(inches);

        return trajectory.build();
    }

    private Trajectory buildLinearTrajectory(Vector2d pos, double heading) {
        DrivetrainController drive = controllers.get(DrivetrainController.class, FieldConstants.Drive);
        drive.turn(Math.toRadians(heading));
        TrajectoryBuilder trajectory = drive.trajectoryBuilder(GetCurrentPose());
        trajectory.lineTo(pos);

        return trajectory.build();
    }


    private Trajectory buildSplineTrajectory(Pose2d[] positions){
        DrivetrainController drive = controllers.get(DrivetrainController.class, FieldConstants.Drive);
        TrajectoryBuilder trajectoryBuilder = drive.trajectoryBuilder(GetCurrentPose());
        for (Pose2d position : positions) {
            trajectoryBuilder.splineTo(position.vec(), Math.toRadians(position.getHeading()));
        }

        Trajectory trajectory = trajectoryBuilder.build();
        return trajectory;
    }

    private Trajectory buildConstantSplineTrajectory(Vector2d position, double heading){
        DrivetrainController drive = controllers.get(DrivetrainController.class, FieldConstants.Drive);
        TrajectoryBuilder trajectory = drive.trajectoryBuilder(GetCurrentPose())
                .splineToConstantHeading(position, Math.toRadians(heading));

        return trajectory.build();
    }

    private Trajectory buildLineTrajectory(Pose2d[] positions){
        DrivetrainController drive = controllers.get(DrivetrainController.class, FieldConstants.Drive);
        TrajectoryBuilder trajectoryBuilder = drive.trajectoryBuilder(GetCurrentPose());
        for (Pose2d position : positions) {
            trajectoryBuilder.lineToLinearHeading(position);
        }

        Trajectory trajectory = trajectoryBuilder.build();
        return trajectory;
    }

    private Trajectory buildLineTrajectory(Pose2d position){
        DrivetrainController drive = controllers.get(DrivetrainController.class, FieldConstants.Drive);
        TrajectoryBuilder trajectory = drive.trajectoryBuilder(GetCurrentPose());
        trajectory.lineToLinearHeading(position);

        return trajectory.build();
    }

    private Trajectory buildStrafeTrajectory(Vector2d position){
        DrivetrainController drive = controllers.get(DrivetrainController.class, FieldConstants.Drive);
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

        DrivetrainController drive = controllers.get(DrivetrainController.class, FieldConstants.Drive);
        drive.followTrajectoryAsync(trajectory);
        drive.waitForIdle();
        //although this may look equivalent to followTrajectory it is non-blocking
        //because sequences run on a separate thread, drive methods do not
    }


    //TODO: Reverse trajectories, pass in false to the builder!
    private Trajectory buildIntakeTrajectory(Vector2d position, double heading, double timeDelay){
        DrivetrainController drive = controllers.get(DrivetrainController.class, FieldConstants.Drive);
        IntakeController intake = controllers.get(IntakeController.class, FieldConstants.Intake);
        drive.turn(heading);
        Trajectory trajectory = drive.trajectoryBuilder(GetCurrentPose())
                        //this ugly thing lowers the speed of our driving
                        .lineToLinearHeading(new Pose2d(position, heading),
                                new MinVelocityConstraint(Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(12, DriveConstants.TRACK_WIDTH)
                                )
                                ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .addTemporalMarker(timeDelay, () -> { // This marker runs x # of seconds into the trajectory
                        intake.run(FORWARD);
                })
                .build();
        return trajectory;
    }

    private Trajectory buildWobbleTrajectory(Vector2d pos, double heading) {
        //slower version of buildLinearTrajectory
        DrivetrainController drive = controllers.get(DrivetrainController.class, FieldConstants.Drive);
        drive.turn(Math.toRadians(heading));
        Trajectory trajectory = drive.trajectoryBuilder(GetCurrentPose())
                .lineTo(pos,
                        new MinVelocityConstraint(Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(7.5, DriveConstants.TRACK_WIDTH)
                        )
                        ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        return trajectory;
    }

    private void turn(double heading){
        DrivetrainController drive = controllers.get(DrivetrainController.class, FieldConstants.Drive);
        drive.turnAsync(Math.toRadians(heading));
        drive.waitForIdle();
    }
}
