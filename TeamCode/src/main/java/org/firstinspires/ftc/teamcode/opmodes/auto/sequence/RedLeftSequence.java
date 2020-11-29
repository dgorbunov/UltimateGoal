package org.firstinspires.ftc.teamcode.opmodes.auto.sequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.auto.Constants;
import org.firstinspires.ftc.teamcode.opmodes.auto.FullAuto;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;


public class RedLeftSequence extends Sequence {

    public RedLeftSequence(ControllerManager controllers, Telemetry tel){
        super(controllers, tel);
        this.startPose = new Pose2d(
                Constants.RedLeft.StartingPos,
                Math.toRadians(0));
    }

    protected void makeActions() {
        switch (ringCount) {
            case 0:
                targetZone = Constants.RedField.TargetZoneA;
                break;
            case 1:
                targetZone = Constants.RedField.TargetZoneB;
                break;
            case 4:
                targetZone = Constants.RedField.TargetZoneC;
                break;
        }
        actions.add(() -> moveToZone(targetZone, 0, 0));
        actions.add(() -> dropWobble());
        actions.add(() -> moveToStart(Constants.RedRight.RightWobblePos, 0,180));
        actions.add(() -> pickupWobble());
        actions.add(() -> moveToZone(targetZone, 180, 0));
        actions.add(() -> moveToShoot(Constants.RedLeft.ShootingPos, 0));
        actions.add(() -> shootRings());
        actions.add(() -> intakeRings(ringCount, Constants.RedField.RingPos, 0));
        actions.add(() -> shootRings());
        actions.add(() -> moveToLaunchLine(Constants.RedLeft.LaunchLine));
        actions.add(() -> stop());

    }
}
