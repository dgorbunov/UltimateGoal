package org.firstinspires.ftc.teamcode.opmodes.auto.sequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;
import org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;
import org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController;
import org.firstinspires.ftc.teamcode.robot.systems.IntakeController;
import org.firstinspires.ftc.teamcode.robot.systems.ShooterController;
import org.firstinspires.ftc.teamcode.robot.systems.WobbleController;

import java.util.Arrays;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

public abstract class Sequence {

    protected int ringCount;
    protected Telemetry telemetry;
    protected ControllerManager controllers;
    protected final static Object lock = new Object();
    protected Actions actions;
    protected Pose2d startPose;
    protected Vector2d targetZone;
    protected TrajectoryBuilder trajectory;

    //TODO: Build all trajectories before running?

    public Sequence(ControllerManager controllers, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.controllers = controllers;
        this.actions = new Actions(this.telemetry);
    }

    public void init(int ringCount) {
        //TODO: move this before init, don't block
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

    public void moveLinear(Vector2d posititon, double targetHeading) {
        trajectory.followTrajectoryAsync(buildLinearTrajectory(posititon, targetHeading));
    }

    public void moveLinear(double x, double y, double targetHeading) {
        trajectory.followTrajectoryAsync(buildLinearTrajectory(new Vector2d(x, y), targetHeading));
    }


    public void moveToZone(Vector2d targetZone, Vector2d intermediatePos, double targetHeading) throws IllegalArgumentException {
        telemetry.addData("Sequence", "moveToZone");
        turn(targetHeading);


        Pose2d positions[] = new Pose2d[] {
                new Pose2d(intermediatePos, targetHeading),
                new Pose2d(targetZone, targetHeading),
        };

        trajectory.followTrajectoryAsync(trajectory.buildSplineTrajectory(positions));
    }

    public void dropWobble() {
        telemetry.addData("Sequence","dropWobble");
        WobbleController wobble = controllers.get(WobbleController.class, FieldConstants.Wobble);
        wobble.dropAuto();
    }

    public void moveToWobble(Vector2d intermediate, Vector2d wobblePos, double endTangent) {
        telemetry.addData("Sequence","moveToWobble");


        trajectory.followTrajectoryAsync(trajectory.buildSplineTrajectory(180, new Pose2d(intermediate, endTangent)));
        trajectory.followTrajectoryAsync(buildLineTrajectory(new Pose2d(wobblePos, endTangent), 12));
    }

    public void pickupWobble() {
        telemetry.addData("Sequence", "collectWobble");
        WobbleController wobble = controllers.get(WobbleController.class, FieldConstants.Wobble);
        wobble.pickupAuto();
    }

    public void moveToShoot(Vector2d intermediate, Vector2d position, double targetHeading) {
        telemetry.addData("Sequence","moveToShoot" );

        Vector2d positions[] = new Vector2d[] {
                intermediate,
                position
        };

//        trajectory.followTrajectoryAsync(buildLineTrajectory(positions));
        trajectory.followTrajectoryAsync(trajectory.buildSplineTrajectoryConstantHeading(positions, targetHeading));
        //TODO: fix, path continuity exception
    }

    public void moveToShoot(Pose2d position, double heading) {
        telemetry.addData("Sequence", "moveToShoot");
        trajectory.followTrajectoryAsync(buildLineTrajectory(position));
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
        trajectory.followTrajectoryAsync(buildBackTrajectory(distance));
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
                trajectory.followTrajectoryAsync(buildIntakeTrajectory(position, heading, 0.1));
                break;

            case (4):
                telemetry.addData("Sequence", "intake 4 rings");
                intake.extend();
                trajectory.followTrajectoryAsync(buildIntakeTrajectory(position, heading, 0.1));
                break;
            default:
                telemetry.addData("Sequence", "unsupported # of rings to intake");

        }
    }

    public void stopIntake(){
        IntakeController intake = controllers.get(IntakeController.class, FieldConstants.Intake);
        intake.stopIntake();
    }

    public void moveToLaunchLine(Vector2d position) {
        telemetry.addData("Sequence","moveToLaunchLine" );
        trajectory.followTrajectoryAsync(buildLineTrajectory(new Pose2d(position, 0)));
    }


    private void turn(double heading){
        DrivetrainController drive = controllers.get(DrivetrainController.class, FieldConstants.Drive);
        drive.turnAsync(Math.toRadians(heading));
        drive.waitForIdle();
    }
}
