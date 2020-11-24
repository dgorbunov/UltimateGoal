package org.firstinspires.ftc.teamcode.opmodes.auto.sequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.auto.Constants;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;

public class RedRightSequence extends Sequence {

    public RedRightSequence(ControllerManager controllers, Telemetry tel){
        super(controllers, tel);
        this.startPose = new Pose2d(
                Constants.RedRight.StartingPos,
                Math.toRadians(0));
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
