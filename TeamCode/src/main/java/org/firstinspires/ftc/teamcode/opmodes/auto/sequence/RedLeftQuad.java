package org.firstinspires.ftc.teamcode.opmodes.auto.sequence;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController;

public class RedLeftQuad implements Sequence {

    DrivetrainController drive;

    public RedLeftQuad (DrivetrainController drive){
        this.drive = drive;
    }

    public SequenceFollower sequenceFollower = new SequenceFollower(drive);
    @Override
    public void run() {
        sequenceFollower.moveToSquares(new Pose2d(0,0,0));
    }

    @Override
    public void moveToSquares() {

    }

    @Override
    public void dropWobble() {
        dropWobble.run();
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
