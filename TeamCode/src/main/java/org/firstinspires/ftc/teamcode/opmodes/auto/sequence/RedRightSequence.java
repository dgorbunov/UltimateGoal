package org.firstinspires.ftc.teamcode.opmodes.auto.sequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;

import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField;
import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedRight;


//import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;

public class RedRightSequence extends Sequence {

    public RedRightSequence(ControllerManager controllers, Telemetry telemetry){
        super(controllers, telemetry);
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
            default:
                try {
                    double x = targetZone.getX();
                } catch (Exception e) {
                    e.printStackTrace();
                    telemetry.addLine(e.toString());
                    telemetry.update();
                }
        }

    }
}
