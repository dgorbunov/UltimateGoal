package org.firstinspires.ftc.teamcode.opmodes.auto.sequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;

import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField;
import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField.EndingPosition;
import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField.PowerShootingPos;
import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField.SideWobbleXOffset;
import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField.SideWobbleYOffset;
import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedLeft;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.RPMAuto;

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

        /**
         * Begin Sequence
         */

        actions.add(() -> moveSplineCustomSpeed(PowerShootingPos, 0, 0, -45, 0.75));
        actions.add(() -> powerShot(MechConstants.RPMPowerShot));

        //TODO: TUNE LATERAL MULTIPLIER/XY MULTIPLIERS
        //TODO: LINEAR INSTEAD OF SPLINE
        //TODO: LINEAR HEADING VS. SPLINE HEADING? CHECK IN SPLINETEST CLASS

        if (ringCount == 1) {
            actions.add(() -> intakeRings(ringCount, RedField.IntakeOnePos, 0));
            actions.add(() -> shootSequence(RedField.GoalShotPos, 0, RPMAuto, 1));
        }
        else if (ringCount == 4) {
            actions.add(() -> intakeRings(ringCount, RedField.IntakeFourPos, 0));
            actions.add(() -> shootSequence(RedField.GoalShotPos, 0, RPMAuto, 3));
        }

        actions.add(() -> stopShooter());

        actions.add(() -> moveLinear(targetZone.getX() + SideWobbleXOffset, targetZone.getY() + SideWobbleYOffset,0));
        actions.add(() -> dropWobbleSide());

        actions.add(() -> moveToWobble(RedField.RightWobbleIntermediate, ringCount));
        actions.add(() -> approachWobble(RedField.RightWobblePos));
        actions.add(() -> pickupWobble());

        actions.add(() -> moveToDropWobble(targetZone, 0.85));
        actions.add(() -> dropWobble());

        if (ringCount == 4) {
            actions.add(() -> moveToLaunchLine(RedField.EndingPositionFour));
        } else if (ringCount == 1){
            actions.add(() -> moveToLaunchLine(RedField.EndingPosition));
        } else {
            actions.add(() -> strafe(drive.getPoseEstimate().getX() - 8, drive.getPoseEstimate().getY()));
            actions.add(() -> strafe(drive.getPoseEstimate().getX(), EndingPosition.getY()));
            actions.add(() -> moveToLaunchLine(RedField.EndingPosition));
        }

        actions.add(() -> stop());
    }
}
