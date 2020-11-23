package org.firstinspires.ftc.teamcode.opmodes.auto.sequence;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;

public class ExampleSequence extends Sequence {
    public ExampleSequence(int ringCount, ControllerManager controllers, HardwareMap hwMap, Telemetry tel){
        super(ringCount, controllers, hwMap, tel);
    }

    @Override
    public boolean execute() throws InterruptedException {

        if (!super.execute()) {
            return false;
        }

        moveToSquares();
        dropWobble();
        moveToStart();
        collectWobble();
        moveToShoot();
        shootRings();
        intakeRings();
        moveToLaunchLine();

        return true;
    }
}
