package org.firstinspires.ftc.teamcode.opmodes.auto.sequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

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
        actions.add(() -> moveToShoot(RedRight.IntermediatePos, new Vector2d(RedField.GoalShotPos.getX(),RedField.GoalShotPos.getY()), 0));
        actions.add(() -> shootGoal(3));

        if (ringCount == 1) {
            actions.add(() -> intakeRings(ringCount, RedField.IntakePos, 0));
            actions.add(() -> moveToShoot(RedField.GoalShotPos, 0));
            actions.add(() -> stopIntake());
            actions.add(() -> shootGoal(1));
        }

        if (ringCount == 4) {
            actions.add(() -> intakeRings(ringCount, RedField.IntakeFourPos, 0));
            actions.add(() -> moveToShoot(RedField.GoalShotPos, 0));
            actions.add(() -> stopIntake());
            actions.add(() -> shootGoal(3));
        }

//        actions.add(() -> stopShooter());

        actions.add(() -> moveLinear(targetZone.getX() + WobbleXOffset, targetZone.getY(),0));
        actions.add(() -> dropWobble());

        actions.add(() -> moveToWobble(RedField.LeftWobbleIntermediate, RedField.LeftWobblePos,180));
        actions.add(() -> pickupWobble());

        actions.add(() -> moveLinear(targetZone, 180));
        actions.add(() -> dropWobble());
        actions.add(() -> backOffFromWobbles(WobbleBackupDistance));

        if (ringCount == 4) {
            actions.add(() -> moveToLaunchLine(RedField.EndingPositionFour));
        } else actions.add(() -> moveToLaunchLine(RedField.EndingPosition));
        actions.add(() -> stop());

    }
}
