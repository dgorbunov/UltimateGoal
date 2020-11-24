package org.firstinspires.ftc.teamcode.opmodes.auto.sequence;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;

public class ExampleSequence extends Sequence {
    public ExampleSequence(ControllerManager controllers, Telemetry tel){
        super(controllers, tel);
    }

    protected void makeActions() {
//        actions.addAction(this::moveToZone);
        actions.add(this::dropWobble);
        actions.add(this::moveToStart);
        actions.add(this::collectWobble);
        actions.add(this::intakeRings);
        actions.add(this::moveToShoot);
        actions.add(this::shootRings);
        actions.add(this::moveToLaunchLine);
    }
}
