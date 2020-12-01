package org.firstinspires.ftc.teamcode.opmodes.auto.sequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.auto.FieldConstants;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;

public class BlueRightSequence extends Sequence {

    public BlueRightSequence(ControllerManager controllers, Telemetry tel) {
        super(controllers, tel);
        this.startPose = new Pose2d(
                FieldConstants.BlueRight.StartingPos,
                Math.toRadians(0)); // TODO: rotate 180?
    }

    protected void makeActions() {
//        switch (ringCount) {
//            case 0:
//                actions.addAction(() -> moveToZone());
//                break;
//            case 1:
//                actions.addAction(() -> moveToZone());
//                break;
//            case 4:
//                actions.addAction(() -> moveToZone());
//                break;
//        }
    }
}
