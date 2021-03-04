package org.firstinspires.ftc.teamcode.robot.drive.tuning;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DrivetrainController drive = new DrivetrainController(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj = new TrajectoryBuilder(new Pose2d(), DrivetrainController.getMaxAngVelConstraint(), DrivetrainController.getMaxAccelConstraint())
                .splineTo(new Vector2d(30, 30), 0)
                .build();

        drive.followTrajectory(traj);

        sleep(2000);

        drive.followTrajectory(
                new TrajectoryBuilder(traj.end(), DrivetrainController.getMaxAngVelConstraint(), DrivetrainController.getMaxAccelConstraint())
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        );
    }
}
