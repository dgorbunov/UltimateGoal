package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;

import org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController;
import org.firstinspires.ftc.teamcode.robot.drive.params.DriveConstants;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController.getMaxAccelConstraint;
import static org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController.getMaxAngVelConstraint;
import static org.firstinspires.ftc.teamcode.robot.drive.params.DriveConstants.MAX_VEL;

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

    public static Trajectory buildSplineTrajectory(DrivetrainController drive, Pose2d... positions){
        TrajectoryBuilder trajectoryBuilder = new TrajectoryBuilder(drive.getPoseEstimate(), getMaxAngVelConstraint(), getMaxAccelConstraint());
        for (Pose2d position : positions) {
            trajectoryBuilder.splineTo(position.vec(), Math.toRadians(position.getHeading()));
        }

        Trajectory trajectory = trajectoryBuilder.build();
        return trajectory;
    }

    public static Trajectory buildSplineTrajectory(DrivetrainController drive, double startTangent, Pose2d... positions){
        TrajectoryBuilder trajectoryBuilder = new TrajectoryBuilder(drive.getPoseEstimate(), Math.toRadians(startTangent), getMaxAngVelConstraint(), getMaxAccelConstraint());
        for (Pose2d position : positions) {
            trajectoryBuilder.splineTo(position.vec(), Math.toRadians(position.getHeading()));
        }

        Trajectory trajectory = trajectoryBuilder.build();
        return trajectory;
    }

    public static Trajectory buildSplineHeadingTrajectory(DrivetrainController drive, double startTangent, double endTangent, Pose2d... positions){
        TrajectoryBuilder trajectoryBuilder = new TrajectoryBuilder(drive.getPoseEstimate(), Math.toRadians(startTangent), getMaxAngVelConstraint(), getMaxAccelConstraint());
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

    public static Trajectory buildStrafeTrajectory(DrivetrainController drive, Vector2d position){
        Trajectory trajectory = new TrajectoryBuilder(drive.getPoseEstimate(), getMaxAngVelConstraint(), getMaxAccelConstraint())
                .strafeTo(position)
                .build();
        return trajectory;
    }

    public static Trajectory buildCustomSpeedLineTrajectory(DrivetrainController drive, Vector2d position, double speed){
        Trajectory trajectory = new TrajectoryBuilder(drive.getPoseEstimate(), getMaxAngVelConstraint(), getMaxAccelConstraint())
                .lineTo(position,
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(speed * MAX_VEL, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        return trajectory;
    }

    public static Trajectory buildCustomSpeedLinearTrajectory(DrivetrainController drive, double x, double y, double heading, double speed) {
        Trajectory trajectory = new TrajectoryBuilder(drive.getPoseEstimate(), getMaxAngVelConstraint(), getMaxAccelConstraint())
                .lineToLinearHeading(new Pose2d(x, y, Math.toRadians(heading)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(speed * MAX_VEL, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        return trajectory;
    }

    public static Trajectory buildCustomSpeedLinearTrajectory(DrivetrainController drive, Vector2d pos, double heading, double speed) {
        Trajectory trajectory = new TrajectoryBuilder(drive.getPoseEstimate(), getMaxAngVelConstraint(), getMaxAccelConstraint())
                .lineToLinearHeading(new Pose2d(pos, Math.toRadians(heading)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(speed * MAX_VEL, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(speed * DriveConstants.MAX_ACCEL))
                .build();

        return trajectory;
    }

    public static Trajectory buildCustomSpeedSplineTrajectory(DrivetrainController drive, double x, double y, double heading, double startTangent, double endTangent, double speed) {
        Trajectory trajectory = new TrajectoryBuilder(drive.getPoseEstimate(), Math.toRadians(startTangent), getMaxAngVelConstraint(), getMaxAccelConstraint())
        .splineToSplineHeading(new Pose2d(x,y, Math.toRadians(heading)),
                Math.toRadians(endTangent),
                new MinVelocityConstraint(
                        Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(speed * MAX_VEL, DriveConstants.TRACK_WIDTH)
                        )
                ),
                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        return trajectory;
    }
}
