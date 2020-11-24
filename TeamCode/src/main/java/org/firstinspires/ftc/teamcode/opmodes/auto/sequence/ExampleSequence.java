package org.firstinspires.ftc.teamcode.opmodes.auto.sequence;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;

public class ExampleSequence extends Sequence {
    public ExampleSequence(ControllerManager controllers, HardwareMap hwMap, Telemetry tel){
        super(controllers, hwMap, tel);
    }

    public void execute(){
        actions.run();
    }

    protected void makeActions() {
//        actions.addAction(() -> moveToZone());
        actions.add(() -> dropWobble());
        actions.add(() -> moveToStart());
        actions.add(() -> collectWobble());

        actions.add(() -> intakeRings());

        actions.add(() -> moveToShoot());
        actions.add(() -> shootRings());
        actions.add(() -> moveToLaunchLine());
    }
}
