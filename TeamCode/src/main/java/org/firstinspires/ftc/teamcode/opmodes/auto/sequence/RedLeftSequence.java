package org.firstinspires.ftc.teamcode.opmodes.auto.sequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.auto.Constants;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;


public class RedLeftSequence extends Sequence {

    public RedLeftSequence(ControllerManager controllers, Telemetry tel){
        super(controllers, tel);
    }

    public void init() {
        Pose2d startPose = new Pose2d(Constants.RedLeft.StartingPos, Math.toRadians(0));

        super.init(startPose);

        makeActions();
    }

    protected void makeActions() {
        switch (ringCount) {
            case 0:
                actions.add(() -> moveToZone(Constants.RedField.TargetZoneA, Math.toRadians(0)));
                break;
            case 1:
                actions.add(() -> moveToZone(Constants.RedField.TargetZoneB, Math.toRadians(0)));
                break;
            case 4:
                actions.add(() -> moveToZone(Constants.RedField.TargetZoneC, Math.toRadians(0)));
                break;
        }

    }
}
