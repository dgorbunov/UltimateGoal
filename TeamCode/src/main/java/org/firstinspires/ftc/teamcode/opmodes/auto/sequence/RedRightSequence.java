package org.firstinspires.ftc.teamcode.opmodes.auto.sequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.auto.Constants;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;

public class RedRightSequence extends Sequence {

    public RedRightSequence(int ringCount, ControllerManager controllers, HardwareMap hwMap, Telemetry tel){
        super(ringCount, controllers, hwMap, tel);
    }

    @Override
    public void init() {
        Pose2d startPose = new Pose2d(
                Constants.RedRight.StartingPos,
                Math.toRadians(0));

        super.init(startPose);

        makeActions();
    }

    public void execute(){
        actions.run();
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
