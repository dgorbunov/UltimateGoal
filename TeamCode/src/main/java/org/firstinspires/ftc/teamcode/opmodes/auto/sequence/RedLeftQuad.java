package org.firstinspires.ftc.teamcode.opmodes.auto.sequence;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.auto.FieldConstants;
import org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController;

public class RedLeftQuad extends Sequence {

    public RedLeftQuad(HardwareMap hwMap, Telemetry tel){
        super(hwMap, tel);
    }

    @Override
    public void init() {
        Pose2d startPose = new Pose2d(
                FieldConstants.RedLeft.StartingPosX,
                FieldConstants.RedLeft.StartingPosY,
                Math.toRadians(180)); // TODO: rotate 180?

        super.init(startPose);
    }

    @Override
    public boolean execute() throws InterruptedException {
        if (!super.execute()) {
            return false;
        }

        moveToSquares();

        return true;
    }
}
