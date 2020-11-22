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
    public boolean execute() throws InterruptedException {

        if (!super.execute()) {
            return false;
        }

        moveToSquares(GetCurrentPose());
        dropWobble(GetCurrentPose());
        moveToStart(GetCurrentPose());
        collectWobble(GetCurrentPose());
        moveToShoot(GetCurrentPose());
        shootRings();
        intakeRings(GetCurrentPose());
        moveToLaunchLine(GetCurrentPose());

        return true;
    }
}
