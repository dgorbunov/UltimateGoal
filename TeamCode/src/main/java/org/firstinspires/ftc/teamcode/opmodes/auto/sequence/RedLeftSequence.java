package org.firstinspires.ftc.teamcode.opmodes.auto.sequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;

import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField;
import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField.PowerShotPos;
import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField.SideWobbleXOffset;
import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField.SideWobbleYOffset;
import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedLeft;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.RPMGoal;

public class RedLeftSequence extends Sequence {

    public RedLeftSequence(ControllerManager controllers, Telemetry telemetry){
        super(controllers, telemetry);
        startPose = new Pose2d(
                RedLeft.StartingPos,
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

        actions.add(() -> spinUp(MechConstants.RPMPowerShot));
        actions.add(() -> moveSpline(PowerShotPos, 0, 40, -10));
//        actions.add(() -> moveLinear(PowerShotPos, 0));
        actions.add(() -> powerShot(MechConstants.RPMPowerShot));

        //TODO: TUNE LATERAL MULTIPLIER/XY MULTIPLIERS
        //TODO: LINEAR INSTEAD OF SPLINE
        //TODO: LINEAR HEADING VS. SPLINE HEADING? CHECK IN SPLINETEST CLASS

        if (ringCount == 1) {
            actions.add(() -> intakeRings(ringCount, RedField.IntakeOnePos, 0));
            actions.add(() -> shootSequence(RedField.GoalShotPos, 0, RPMGoal, 1));
        }
        else if (ringCount == 4) {
            actions.add(() -> intakeRings(ringCount, RedField.IntakeFourPos, 0));
            actions.add(() -> shootSequence(RedField.GoalShotPos, 0, RPMGoal, 3));
        }

        actions.add(() -> moveLinear(targetZone.getX() + SideWobbleXOffset, targetZone.getY() + SideWobbleYOffset,0));
        actions.add(() -> dropWobbleSide());

        actions.add(() -> moveToWobble(RedField.RightWobbleIntermediate));
        actions.add(() -> approachWobble(RedField.RightWobblePos));
        actions.add(() -> pickupWobble());

        actions.add(() -> moveToDropWobble(targetZone, 0.80));
        actions.add(() -> dropWobble());

        if (ringCount == 4) {
            actions.add(() -> moveToLaunchLine(RedField.EndingPositionFour));
        } else if (ringCount == 1){
            actions.add(() -> moveToLaunchLine(RedField.EndingPosition));
        } else {
            actions.add(() -> strafe(RedField.EndingPosition.getX() - 18, RedField.EndingPosition.getY()));
            actions.add(() -> moveToLaunchLine(RedField.EndingPosition));
        }

        actions.add(() -> stop());
    }
}
