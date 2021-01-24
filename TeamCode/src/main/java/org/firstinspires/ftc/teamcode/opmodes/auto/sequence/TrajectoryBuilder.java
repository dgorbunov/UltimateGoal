package org.firstinspires.ftc.teamcode.opmodes.auto.sequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;
import org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController;
import org.firstinspires.ftc.teamcode.robot.systems.IntakeController;

import java.util.Arrays;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

public class TrajectoryBuilder1 {
    protected Telemetry telemetry;
    protected ControllerManager controllers;

    public TrajectoryBuilder1(ControllerManager controllers, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.controllers = controllers;
    }
/*
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, velConstraint, accelConstraint);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, velConstraint, accelConstraint);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, velConstraint, accelConstraint);
    }
*/
    public Trajectory buildSplineTrajectoryConstantHeading(Vector2d positions[], double targetHeading){
        DrivetrainController drive = controllers.get(DrivetrainController.class, FieldConstants.Drive);
        TrajectoryBuilder trajectory = new TrajectoryBuilder(drive.getPoseEstimate(), drive.getMaxAngVelConstraint(), drive.getMaxAccelConstraint());
        for (Vector2d position : positions) {
            trajectory.splineToConstantHeading(position, Math.toRadians(targetHeading));
        }

        return trajectory.build();
    }

    public Trajectory buildBackTrajectory(double inches) {
        DrivetrainController drive = controllers.get(DrivetrainController.class, FieldConstants.Drive);
        trajectory.TrajectoryBuilder trajectory = drive.trajectoryBuilder(drive.getPoseEstimate());
        trajectory.back(inches);

        return trajectory.build();
    }

    public Trajectory buildLinearTrajectory(Vector2d pos, double heading) {
        DrivetrainController drive = controllers.get(DrivetrainController.class, FieldConstants.Drive);
        drive.turn(Math.toRadians(heading));
        trajectory.TrajectoryBuilder trajectory = drive.trajectoryBuilder(drive.getPoseEstimate());
        trajectory.lineTo(pos);

        return trajectory.build();
    }

    public Trajectory buildLineLinearHeadingTrajectory(Pose2d pos) {
        DrivetrainController drive = controllers.get(DrivetrainController.class, FieldConstants.Drive);
        trajectory.TrajectoryBuilder trajectory = drive.trajectoryBuilder(drive.getPoseEstimate());
        trajectory.lineToLinearHeading(pos);

        return trajectory.build();
    }

    public Trajectory buildSplineTrajectory(Pose2d... positions){
        DrivetrainController drive = controllers.get(DrivetrainController.class, FieldConstants.Drive);
        trajectory.TrajectoryBuilder trajectoryBuilder = drive.trajectoryBuilder(drive.getPoseEstimate());
        for (Pose2d position : positions) {
            trajectoryBuilder.splineTo(position.vec(), Math.toRadians(position.getHeading()));
        }

        Trajectory trajectory = trajectoryBuilder.build();
        return trajectory;
    }

    public Trajectory buildSplineTrajectory(double startTangent, Pose2d... positions){
        DrivetrainController drive = controllers.get(DrivetrainController.class, FieldConstants.Drive);
        trajectory.TrajectoryBuilder trajectoryBuilder = drive.trajectoryBuilder(drive.getPoseEstimate(), Math.toRadians(startTangent));
        for (Pose2d position : positions) {
            trajectoryBuilder.splineTo(position.vec(), Math.toRadians(position.getHeading()));
        }

        Trajectory trajectory = trajectoryBuilder.build();
        return trajectory;
    }

    public Trajectory buildConstantSplineTrajectory(Vector2d[] positions, double heading){
        DrivetrainController drive = controllers.get(DrivetrainController.class, FieldConstants.Drive);
        trajectory.TrajectoryBuilder trajectory = drive.trajectoryBuilder(drive.getPoseEstimate());
        for (Vector2d position : positions) {
            trajectory.splineToConstantHeading(position, Math.toRadians(heading));
        }

        return trajectory.build();
    }

    public Trajectory buildLineTrajectory(Pose2d position){
        DrivetrainController drive = controllers.get(DrivetrainController.class, FieldConstants.Drive);
        trajectory.TrajectoryBuilder trajectory = drive.trajectoryBuilder(drive.getPoseEstimate());
        trajectory.lineToLinearHeading(position);

        return trajectory.build();
    }

    public Trajectory buildLineTrajectory(Pose2d position, double velocity){
        DrivetrainController drive = controllers.get(DrivetrainController.class, FieldConstants.Drive);
        Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(position,
                        new MinVelocityConstraint(Arrays.asList(
                                drive.getMaxAngVelConstraint(),
                                drive.getCustomVelConstraint(velocity)
                        )
                        ), drive.getMaxAccelConstraint())
                .build();

        return trajectory;
    }

    public Trajectory buildStrafeTrajectory(Vector2d position){
        DrivetrainController drive = controllers.get(DrivetrainController.class, FieldConstants.Drive);
        Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeTo(position)
                .build();
        return trajectory;
    }

    //TODO: Reverse trajectories, pass in false to the builder!
    public Trajectory buildIntakeTrajectory(Vector2d position, double heading, double timeDelay){
        DrivetrainController drive = controllers.get(DrivetrainController.class, FieldConstants.Drive);
        IntakeController intake = controllers.get(IntakeController.class, FieldConstants.Intake);
        drive.turn(heading);
        Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .addTemporalMarker(timeDelay, () -> { // This marker runs x # of seconds into the trajectory
                    intake.run(FORWARD);
                })
                //this ugly thing lowers the speed of our driving
                .lineToLinearHeading(new Pose2d(position, heading),
                        new MinVelocityConstraint(Arrays.asList(
                                drive.getMaxAngVelConstraint(),
                                drive.getCustomVelConstraint(15)
                        )
                        ), drive.getMaxAccelConstraint())

                .build();
        return trajectory;
    }

    public Trajectory buildWobbleTrajectory(Vector2d pos, double heading) {
        //slower version of buildLinearTrajectory
        DrivetrainController drive = controllers.get(DrivetrainController.class, FieldConstants.Drive);
        drive.turn(Math.toRadians(heading));
        Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineTo(pos,
                        new MinVelocityConstraint(Arrays.asList(
                                drive.getMaxAngVelConstraint(),
                                drive.getCustomVelConstraint(7)
                        )
                        ), drive.getMaxAccelConstraint())
                .build();

        return trajectory;
    }
}
