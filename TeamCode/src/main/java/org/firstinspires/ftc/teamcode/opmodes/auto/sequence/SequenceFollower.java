package org.firstinspires.ftc.teamcode.opmodes.auto.sequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.drive.SampleMecanumDrive;

public class SequenceFollower implements Sequence {

    Telemetry telemetry;
    SampleMecanumDrive drive;

    public SequenceFollower(SampleMecanumDrive drive, Telemetry telemetry){
        this.drive = drive;
        this.telemetry = telemetry;
    }

    @Override
    public void run() {

    }

    @Override
    public void moveToSquares(Pose2d position) {
        Trajectory mySequence = drive.trajectoryBuilder(position)
                .strafeRight(10)
                .forward(5)
                .build();

        drive.followTrajectory(mySequence);
    }

    @Override
    public void dropWobble(Pose2d position) {



    }

    @Override
    public void moveToStart() {

    }

    @Override
    public void collectWobble() {

    }

    @Override
    public void moveToShoot() {

    }

    @Override
    public void shootRings() {

    }

    @Override
    public void intakeRings() {

    }

    @Override
    public void moveToLaunchLine() {

    }

    @Override
    public void stop() {

    }
}
