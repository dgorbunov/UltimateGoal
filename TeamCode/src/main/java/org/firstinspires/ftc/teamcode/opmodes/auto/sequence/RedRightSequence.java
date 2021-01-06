package org.firstinspires.ftc.teamcode.opmodes.auto.sequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;

public class RedRightSequence extends Sequence {

    public RedRightSequence(ControllerManager controllers, Telemetry tel){
        super(controllers, tel);
        startPose = new Pose2d(
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
        actions.add(() -> startShooter(4800));
        actions.add(() -> moveToShoot(FieldConstants.RedField.ShootingPos, 0));
        actions.add(() -> shootRings(3));

        actions.add(() -> intakeRings(ringCount, FieldConstants.RedField.IntakePos, 0));
        actions.add(() -> moveToShoot(FieldConstants.RedField.ShootingPos, 0));
        actions.add(() -> shootRings(ringCount));
        actions.add(() -> stopShooter());

        actions.add(() -> moveLinear(targetZone, 0));
        actions.add(() -> dropWobble());

        actions.add(() -> moveToStart(FieldConstants.RedLeft.LeftWobblePos,0,180));
        actions.add(() -> pickupWobble());
        actions.add(() -> moveLinear(targetZone, 180));
        actions.add(() -> dropWobble());

        actions.add(() -> moveToLaunchLine(FieldConstants.RedRight.LaunchLine));
        actions.add(() -> stop());

    }
}
