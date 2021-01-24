package org.firstinspires.ftc.teamcode.robot.drive.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController;
import org.firstinspires.ftc.teamcode.robot.drive.params.ThreeWheelLocalizer;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
//@Disabled
@Autonomous(group = "drive")
public class StraightTest extends LinearOpMode {
    public static double DISTANCE = 60; // in

    @Override
    public void runOpMode() throws InterruptedException {
        DrivetrainController drive = new DrivetrainController(hardwareMap);
        ThreeWheelLocalizer odometry = new ThreeWheelLocalizer(hardwareMap);

        Trajectory trajectory = new TrajectoryBuilder(new Pose2d(), DrivetrainController.getMaxAngVelConstraint(), DrivetrainController.getMaxAccelConstraint())
                .forward(DISTANCE)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.addData("left encoder:", odometry.getWheelPositions().get(0));
        telemetry.addData("right encoder:", odometry.getWheelPositions().get(1));
        telemetry.addData("front encoder:", odometry.getWheelPositions().get(2));
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
