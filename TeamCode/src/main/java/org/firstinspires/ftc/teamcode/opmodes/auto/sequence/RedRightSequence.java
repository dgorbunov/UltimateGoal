package org.firstinspires.ftc.teamcode.opmodes.auto.sequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;

import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField;
import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField.WobbleBackupDistance;
import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField.WobbleXOffset;
import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedRight;

//import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;

public class RedRightSequence extends Sequence {

    public RedRightSequence(ControllerManager controllers, Telemetry tel){
        super(controllers, tel);
        startPose = new Pose2d(
                RedRight.StartingPos,
                Math.toRadians(0));

    }

    protected void makeActions() {
        switch (ringCount) {
            case 0:
                targetZone = RedField.TargetZoneA;
                break;
            case 1:
                targetZone = RedField.TargetZoneB;
                break;
            case 4:
                targetZone = RedField.TargetZoneC;
                break;
        }
//        actions.add(() -> startShooter(RPMGoal));
        actions.add(() -> moveToShoot(RedRight.IntermediatePos, RedField.GoalShotPos));
        actions.add(() -> shootGoal(3));

        if (ringCount != 0) {
            actions.add(() -> intakeRings(ringCount, RedField.IntakePos, 0));
            actions.add(() -> moveToShoot(RedField.GoalShotPos, 0));
            actions.add(() -> shootGoal(ringCount));
        }

        actions.add(() -> stopShooter());

        actions.add(() -> moveLinear(targetZone.getX() + WobbleXOffset, targetZone.getY(),0));
        actions.add(() -> dropWobble());

        actions.add(() -> moveToWobble(RedField.LeftWobbleIntermediate, RedField.LeftWobblePos,180));
        actions.add(() -> pickupWobble());

        actions.add(() -> moveLinear(targetZone, 180));
        actions.add(() -> dropWobble());
        actions.add(() -> backOffFromWobbles(WobbleBackupDistance));

        actions.add(() -> moveToLaunchLine(RedField.EndingPosition));
        actions.add(() -> stop());

    }
}
