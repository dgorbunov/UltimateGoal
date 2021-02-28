package org.firstinspires.ftc.teamcode.robot.drive.tuning;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;
import org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class CompoundSplineTest extends LinearOpMode {
    public static double DISTANCE = 60; // in
    public static double ANGLE = 180;

    @Override
    public void runOpMode() throws InterruptedException {
        DrivetrainController drive = new DrivetrainController(hardwareMap);
        Pose2d startingPos = new Pose2d(FieldConstants.RedRight.StartingPos, Math.toRadians(0));
        drive.setPoseEstimate(startingPos);

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive())  {
            drive.followTrajectory(new TrajectoryBuilder(drive.getPoseEstimate(), DrivetrainController.getMaxAngVelConstraint(), DrivetrainController.getMaxAccelConstraint())
                    .lineToLinearHeading(new Pose2d(startingPos.getX() + DISTANCE, startingPos.getY(), Math.toRadians(ANGLE)))
                    .build());
            sleep(200);
            drive.followTrajectory(new TrajectoryBuilder(drive.getPoseEstimate(), DrivetrainController.getMaxAngVelConstraint(), DrivetrainController.getMaxAccelConstraint())
                    .lineToLinearHeading(new Pose2d(startingPos.getX(), startingPos.getY(), Math.toRadians(0)))
                    .build());
            sleep(200);
        };
    }
}
