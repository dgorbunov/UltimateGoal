package org.firstinspires.ftc.teamcode.opmodes.auto.sequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.auto.FieldConstants;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;

public class RedRightSequence extends Sequence {

    public RedRightSequence(ControllerManager controllers, Telemetry tel){
        super(controllers, tel);
        this.startPose = new Pose2d(
                FieldConstants.RedRight.StartingPos,
                Math.toRadians(0));
    }

    protected void makeActions() {
        switch (ringCount) {
            case 0:
                targetZone = FieldConstants.RedField.TargetZoneA;
                break;
            case 1:
                targetZone = FieldConstants.RedField.TargetZoneB;
                break;
            case 4:
                targetZone = FieldConstants.RedField.TargetZoneC;
                break;
        }
        actions.add(() -> moveToZone(targetZone, FieldConstants.RedRight.IntermediatePos, 0, 0));
        actions.add(() -> dropWobble());
        actions.add(() -> moveToStart(FieldConstants.RedLeft.LeftWobblePos, FieldConstants.RedLeft.IntermediatePos, 0,180));
        actions.add(() -> pickupWobble());
        actions.add(() -> moveToZone(targetZone, FieldConstants.RedLeft.IntermediatePos,  180, 0));
        actions.add(() -> moveToShoot(FieldConstants.RedField.ShootingPos, 0));
        actions.add(() -> shootRings());
        actions.add(() -> intakeRings(ringCount, FieldConstants.RedField.IntakePos, 0));
        actions.add(() -> shootRings());
        actions.add(() -> moveToLaunchLine(FieldConstants.RedRight.LaunchLine));
        actions.add(() -> stop());

    }
}
