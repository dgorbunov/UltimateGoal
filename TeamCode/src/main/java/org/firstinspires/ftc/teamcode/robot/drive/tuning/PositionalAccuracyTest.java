package org.firstinspires.ftc.teamcode.robot.drive.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;
import org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController;

import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField.TargetZoneA;

@Config
@Autonomous(group = "drive")
public class PositionalAccuracyTest extends LinearOpMode {

    public enum StartPos {
        RED_LEFT, RED_RIGHT
    }

    public static StartPos StartPosition = StartPos.RED_LEFT;

    @Override
    public void runOpMode() throws InterruptedException {
        DrivetrainController drive = new DrivetrainController(hardwareMap);
        if (StartPosition == StartPos.RED_LEFT) drive.setPoseEstimate(new Pose2d(FieldConstants.RedLeft.StartingPos, Math.toRadians(0)));
        else drive.setPoseEstimate(new Pose2d(FieldConstants.RedRight.StartingPos, Math.toRadians(0)));


        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj = new TrajectoryBuilder(drive.getPoseEstimate(), DrivetrainController.getMaxAngVelConstraint(), DrivetrainController.getMaxAccelConstraint())
                .splineTo(TargetZoneA, 0)
                .build();

        drive.followTrajectory(traj);

//        sleep(2000);
//
//        drive.followTrajectory(
//                new TrajectoryBuilder(traj.end(), true, DrivetrainController.getMaxAngVelConstraint(), DrivetrainController.getMaxAccelConstraint())
//                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
//                        .build()
//        );
    }
}
