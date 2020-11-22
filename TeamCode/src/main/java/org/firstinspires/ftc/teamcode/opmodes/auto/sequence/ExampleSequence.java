package org.firstinspires.ftc.teamcode.opmodes.auto.sequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.drive.SampleMecanumDrive;

public class ExampleSequence extends Sequence {
    public ExampleSequence(HardwareMap hwMap, Telemetry tel){
        super(hwMap, tel);
    }

    @Override
    public void init() {

    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
    }


    public void run() {
        Pose2d pos = new Pose2d(0,0);
        moveToSquares(pos);
        dropWobble(pos);
        moveToStart(pos);
        collectWobble(pos);
        moveToShoot(pos);
        shootRings(pos);
        intakeRings(pos);
        moveToLaunchLine(pos);
        stop();
    }
}
