package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;

import org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController.getCustomAccelConstraint;
import static org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController.getCustomMaxAngVelConstraint;
import static org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController.getCustomVelConstraint;
import static org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController.getMaxAccelConstraint;
import static org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController.getMaxAngVelConstraint;

public class TrajectoryHelper {

    private static Trajectory trajectory1;
    private static Trajectory trajectory2;

    public static Trajectory buildSplineTrajectoryConstantHeading(DrivetrainController drive, Vector2d positions[], double targetHeading){
        TrajectoryBuilder trajectory = new TrajectoryBuilder(drive.getPoseEstimate(), getMaxAngVelConstraint(), getMaxAccelConstraint());
        for (Vector2d position : positions) {
            trajectory.splineToConstantHeading(position, Math.toRadians(targetHeading));
        }

        return trajectory.build();
    }

    public static Trajectory buildBackTrajectory(DrivetrainController drive, double inches) {
        TrajectoryBuilder trajectory = new TrajectoryBuilder(drive.getPoseEstimate(), getMaxAngVelConstraint(), getMaxAccelConstraint());
        trajectory.back(inches);

        return trajectory.build();
    }

    public static Trajectory buildLinearTrajectory(DrivetrainController drive, double x, double y, double heading) {
        TrajectoryBuilder trajectory = new TrajectoryBuilder(drive.getPoseEstimate(), getMaxAngVelConstraint(), getMaxAccelConstraint());
        trajectory.lineToLinearHeading(new Pose2d(x, y, Math.toRadians(heading)));

        return trajectory.build();
    }

    public static Trajectory buildLinearChainTrajectory(DrivetrainController drive, Pose2d position, Pose2d position2) {
        Trajectory trajectory1 = new TrajectoryBuilder(drive.getPoseEstimate(), getMaxAngVelConstraint(), getMaxAccelConstraint())
                .lineToLinearHeading(position)
                .addDisplacementMarker(() -> drive.followTrajectory(trajectory2))
                .build();

        //TODO: this is broken
        Trajectory trajectory2 = new TrajectoryBuilder(trajectory1.end(), getMaxAngVelConstraint(), getMaxAccelConstraint())
                .lineToLinearHeading(position2)
                .build();

        return trajectory1;
    }

    public static Trajectory buildAutoShootTrajectory(DrivetrainController drive, Pose2d pos, double velocity, double acceleration) {
        Trajectory trajectory = new TrajectoryBuilder(drive.getPoseEstimate(), getMaxAngVelConstraint(), getMaxAccelConstraint())
                .lineToLinearHeading(pos,
                        new MinVelocityConstraint(Arrays.asList(
                                drive.getMaxAngVelConstraint(),
                                getCustomVelConstraint(velocity)
                        )
                        ), getCustomAccelConstraint(acceleration))

                .build();

        return trajectory;
    }


    public static Trajectory buildSplineTrajectory(DrivetrainController drive, Pose2d... positions){
        TrajectoryBuilder trajectoryBuilder = new TrajectoryBuilder(drive.getPoseEstimate(), getMaxAngVelConstraint(), getMaxAccelConstraint());
        for (Pose2d position : positions) {
            trajectoryBuilder.splineTo(position.vec(), Math.toRadians(position.getHeading()));
        }

        Trajectory trajectory = trajectoryBuilder.build();
        return trajectory;
    }

    public static Trajectory buildSplineTrajectory(DrivetrainController drive, double startTangent, Pose2d... positions){
        TrajectoryBuilder trajectoryBuilder = new TrajectoryBuilder(drive.getPoseEstimate(), startTangent, getMaxAngVelConstraint(), getMaxAccelConstraint());
        for (Pose2d position : positions) {
            trajectoryBuilder.splineTo(position.vec(), Math.toRadians(position.getHeading()));
        }

        Trajectory trajectory = trajectoryBuilder.build();
        return trajectory;
    }

    public static Trajectory buildSplineHeadingTrajectory(DrivetrainController drive, double startTangent, double endTangent, Pose2d... positions){
        TrajectoryBuilder trajectoryBuilder = new TrajectoryBuilder(drive.getPoseEstimate(), startTangent, getMaxAngVelConstraint(), getMaxAccelConstraint());
        for (Pose2d position : positions) {
            trajectoryBuilder.splineToSplineHeading(new Pose2d(position.vec(), Math.toRadians(position.getHeading())), Math.toRadians(endTangent));
        }

        Trajectory trajectory = trajectoryBuilder.build();
        return trajectory;
    }

    public static Trajectory buildSplineLinearHeadingTrajectory(DrivetrainController drive, double startTangent, double endTangent, Pose2d... positions){
        TrajectoryBuilder trajectoryBuilder = new TrajectoryBuilder(drive.getPoseEstimate(), Math.toRadians(startTangent), getMaxAngVelConstraint(), getMaxAccelConstraint());
        for (Pose2d position : positions) {
            trajectoryBuilder.splineToLinearHeading(new Pose2d(position.vec(), Math.toRadians(position.getHeading())), Math.toRadians(endTangent));
        }

        Trajectory trajectory = trajectoryBuilder.build();
        return trajectory;
    }

    public static Trajectory buildLineTrajectory(DrivetrainController drive, double x, double y){
        TrajectoryBuilder trajectory = new TrajectoryBuilder(drive.getPoseEstimate(), getMaxAngVelConstraint(), getMaxAccelConstraint());
        trajectory.lineTo(new Vector2d(x,y));

        return trajectory.build();
    }

    public static Trajectory buildLinearTrajectory(DrivetrainController drive, Pose2d position, double vel, double angVel, double accel){
        Trajectory trajectory = new TrajectoryBuilder(drive.getPoseEstimate(), getMaxAngVelConstraint(), getMaxAccelConstraint())
                .lineToLinearHeading(new Pose2d(position.vec(), Math.toRadians(position.getHeading())),
                        new MinVelocityConstraint(Arrays.asList(
                                getCustomMaxAngVelConstraint(angVel),
                                getCustomVelConstraint(vel)
                        )
                        ), getCustomAccelConstraint(accel))
                .build();

        return trajectory;
    }

    public static Trajectory buildStrafeTrajectory(DrivetrainController drive, Vector2d position){
        Trajectory trajectory = new TrajectoryBuilder(drive.getPoseEstimate(), getMaxAngVelConstraint(), getMaxAccelConstraint())
                .strafeTo(position)
                .build();
        return trajectory;
    }

    public static Trajectory buildIntakeTrajectory(DrivetrainController drive, Vector2d position, double speed){
        Trajectory trajectory = new TrajectoryBuilder(drive.getPoseEstimate(), getMaxAngVelConstraint(), getMaxAccelConstraint())
                //this ugly thing lowers the speed of our driving
                .lineTo(position,
                        new MinVelocityConstraint(Arrays.asList(
                                getMaxAngVelConstraint(),
                                getCustomVelConstraint(speed)
                        )
                        ), getMaxAccelConstraint())

                .build();
        return trajectory;
    }

    public static Trajectory buildPowerShotTrajectory(DrivetrainController drive, Vector2d position, double heading){
        Trajectory trajectory = new TrajectoryBuilder(drive.getPoseEstimate(), getMaxAngVelConstraint(), getMaxAccelConstraint())
                //this ugly thing lowers the speed of our driving
                .lineToLinearHeading(new Pose2d(position, Math.toRadians(heading)),
                        new MinVelocityConstraint(Arrays.asList(
                                getMaxAngVelConstraint(),
                                getCustomVelConstraint(15)
                        )
                        ), getMaxAccelConstraint())

                .build();
        return trajectory;
    }

    public static Trajectory buildCustomSpeedLinearTrajectory(DrivetrainController drive, double x, double y, double heading, double speed) {
        Trajectory trajectory = new TrajectoryBuilder(drive.getPoseEstimate(), getMaxAngVelConstraint(), getMaxAccelConstraint())
                .lineToLinearHeading(new Pose2d(x, y, Math.toRadians(heading)),
                        new MinVelocityConstraint(Arrays.asList(
                                getMaxAngVelConstraint(),
                                getCustomVelConstraint(speed)
                        )
                        ), getMaxAccelConstraint())
                .build();

        return trajectory;
    }
}
