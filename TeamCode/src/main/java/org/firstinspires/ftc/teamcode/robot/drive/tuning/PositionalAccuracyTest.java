package org.firstinspires.ftc.teamcode.robot.drive.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;
import org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField.TargetZoneA;
import static org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController.getCustomAccelConstraint;
import static org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController.getCustomVelConstraint;
import static org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController.getMaxAngVelConstraint;
import static org.firstinspires.ftc.teamcode.robot.drive.params.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.robot.drive.params.DriveConstants.MAX_VEL;

@Config
@Autonomous(group = "drive")
public class PositionalAccuracyTest extends LinearOpMode {

    public enum StartPos {
        RED_LEFT, RED_RIGHT, RED_LOCALIZE_POS
    }

    public static StartPos StartPosition = StartPos.RED_LEFT;

    @Override
    public void runOpMode() throws InterruptedException {
        DrivetrainController drive = new DrivetrainController(hardwareMap);
        if (StartPosition == StartPos.RED_LEFT) drive.setPoseEstimate(new Pose2d(FieldConstants.RedLeft.StartingPos, Math.toRadians(0)));
        else if (StartPosition == StartPos.RED_RIGHT) drive.setPoseEstimate(new Pose2d(FieldConstants.RedRight.StartingPos, Math.toRadians(0)));
        else drive.setPoseEstimate(new Pose2d(FieldConstants.RedField.LocalizePos, Math.toRadians(0)));

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj = new TrajectoryBuilder(drive.getPoseEstimate(), DrivetrainController.getMaxAngVelConstraint(), DrivetrainController.getMaxAccelConstraint())
                .lineTo(TargetZoneA,
                        new MinVelocityConstraint(Arrays.asList(
                                getMaxAngVelConstraint(),
                                getCustomVelConstraint(MAX_VEL * 0.75)
                        )
                        ), getCustomAccelConstraint(MAX_ACCEL * 0.75))

                .build();

        drive.followTrajectory(traj);
    }
}
