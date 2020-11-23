package org.firstinspires.ftc.teamcode.opmodes.auto.sequence;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;

public class ExampleSequence extends Sequence {
    public ExampleSequence(int ringCount, ControllerManager controllers, HardwareMap hwMap, Telemetry tel){
        super(ringCount, controllers, hwMap, tel);
    }

    public void execute(){
        actions.run();
    }

    protected void makeActions() {
//        actions.addAction(() -> moveToZone());
        actions.addAction(() -> dropWobble());
        actions.addAction(() -> moveToStart());
        actions.addAction(() -> collectWobble());

        actions.addAction(() -> intakeRings());

        actions.addAction(() -> moveToShoot());
        actions.addAction(() -> shootRings());
        actions.addAction(() -> moveToLaunchLine());
    }
}
