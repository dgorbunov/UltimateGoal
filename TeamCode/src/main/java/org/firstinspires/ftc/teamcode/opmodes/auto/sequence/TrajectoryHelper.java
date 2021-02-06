package org.firstinspires.ftc.teamcode.opmodes.auto.sequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;
import org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController;
import org.firstinspires.ftc.teamcode.robot.systems.IntakeController;

import java.util.Arrays;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField.GoalShotPos;
import static org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController.getMaxAccelConstraint;
import static org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController.getMaxAngVelConstraint;

public class TrajectoryHelper {

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

    public static Trajectory buildLinearTrajectory(DrivetrainController drive, Vector2d pos, double heading) {
        drive.turn(Math.toRadians(heading));
        TrajectoryBuilder trajectory = new TrajectoryBuilder(drive.getPoseEstimate(), getMaxAngVelConstraint(), getMaxAccelConstraint());
        trajectory.lineTo(pos);

        return trajectory.build();
    }

    public static Trajectory buildAutoShootTrajectory(DrivetrainController drive, Pose2d pos, double velocity) {
        Trajectory trajectory = new TrajectoryBuilder(drive.getPoseEstimate(), getMaxAngVelConstraint(), getMaxAccelConstraint())
                .lineToLinearHeading(GoalShotPos,
                        new MinVelocityConstraint(Arrays.asList(
                                drive.getMaxAngVelConstraint(),
                                drive.getCustomVelConstraint(velocity)
                        )
                        ), drive.getMaxAccelConstraint())

                .build();

        return trajectory;
    }

    public static Trajectory buildLineLinearHeadingTrajectory(DrivetrainController drive, Pose2d pos) {
        TrajectoryBuilder trajectory = new TrajectoryBuilder(drive.getPoseEstimate(), getMaxAngVelConstraint(), getMaxAccelConstraint());
        trajectory.lineToLinearHeading(pos);

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

    public static Trajectory buildConstantSplineTrajectory(DrivetrainController drive, Vector2d[] positions, double heading){
        TrajectoryBuilder trajectory = new TrajectoryBuilder(drive.getPoseEstimate(), getMaxAngVelConstraint(), getMaxAccelConstraint());
        for (Vector2d position : positions) {
            trajectory.splineToConstantHeading(position, Math.toRadians(heading));
        }

        return trajectory.build();
    }

    public static Trajectory buildLineTrajectory(DrivetrainController drive, Pose2d position){
        TrajectoryBuilder trajectory = new TrajectoryBuilder(drive.getPoseEstimate(), getMaxAngVelConstraint(), getMaxAccelConstraint());
        trajectory.lineToLinearHeading(position);

        return trajectory.build();
    }

    public static Trajectory buildLineTrajectory(DrivetrainController drive, Pose2d position, double velocity){
        Trajectory trajectory = new TrajectoryBuilder(drive.getPoseEstimate(), getMaxAngVelConstraint(), getMaxAccelConstraint())
                .lineToLinearHeading(new Pose2d(position.vec(), Math.toRadians(position.getHeading())),
                        new MinVelocityConstraint(Arrays.asList(
                                getMaxAngVelConstraint(),
                                drive.getCustomVelConstraint(velocity)
                        )
                        ), getMaxAccelConstraint())
                .build();

        return trajectory;
    }

    public static Trajectory buildStrafeTrajectory(DrivetrainController drive, Vector2d position){
        Trajectory trajectory = new TrajectoryBuilder(drive.getPoseEstimate(), getMaxAngVelConstraint(), getMaxAccelConstraint())
                .strafeTo(position)
                .build();
        return trajectory;
    }

    //TODO: Reverse trajectories, pass in false to the builder!
    public static Trajectory buildIntakeTrajectory(DrivetrainController drive, IntakeController intake, Vector2d position, double heading, double timeDelay){
        drive.turn(heading);
        Trajectory trajectory = new TrajectoryBuilder(drive.getPoseEstimate(), getMaxAngVelConstraint(), getMaxAccelConstraint())
                .addTemporalMarker(timeDelay, () -> { // This marker runs x # of seconds into the trajectory
                    intake.run(FORWARD);
                })
                //this ugly thing lowers the speed of our driving
                .lineToLinearHeading(new Pose2d(position, heading),
                        new MinVelocityConstraint(Arrays.asList(
                                getMaxAngVelConstraint(),
                                drive.getCustomVelConstraint(15)
                        )
                        ), getMaxAccelConstraint())

                .build();
        return trajectory;
    }

    public static Trajectory buildWobbleTrajectory(DrivetrainController drive, Vector2d pos, double heading) {
        //slower version of buildLinearTrajectory
        drive.turn(Math.toRadians(heading));
        Trajectory trajectory = new TrajectoryBuilder(drive.getPoseEstimate(), getMaxAngVelConstraint(), getMaxAccelConstraint())
                .lineTo(pos,
                        new MinVelocityConstraint(Arrays.asList(
                                getMaxAngVelConstraint(),
                                drive.getCustomVelConstraint(7)
                        )
                        ), getMaxAccelConstraint())
                .build();

        return trajectory;
    }
}
