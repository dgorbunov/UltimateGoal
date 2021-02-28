package org.firstinspires.ftc.teamcode.robot.drive.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController;

/**
 * Used to manually calibrate LATERAL_MULTIPLIER in DrivetrainController
 * This is used to adequately model robot behavior in feedforward control
 * As mecanum chassis exhibit less torque when moving laterally (strafing)
 */

@Config
@Autonomous(group = "drive")
public class StrafeBackAndForth extends LinearOpMode {

    public static double DISTANCE = 65;

    @Override
    public void runOpMode() throws InterruptedException {
        DrivetrainController drive = new DrivetrainController(hardwareMap);

        Trajectory trajectoryForward = new TrajectoryBuilder(new Pose2d(), DrivetrainController.getMaxAngVelConstraint(), DrivetrainController.getMaxAccelConstraint())
                .strafeRight(DISTANCE)
                .build();

        Trajectory trajectoryBackward = new TrajectoryBuilder(trajectoryForward.end(), DrivetrainController.getMaxAngVelConstraint(), DrivetrainController.getMaxAccelConstraint())
                .strafeLeft(DISTANCE)
                .build();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drive.followTrajectory(trajectoryForward);
            drive.followTrajectory(trajectoryBackward);
        }
    }
}