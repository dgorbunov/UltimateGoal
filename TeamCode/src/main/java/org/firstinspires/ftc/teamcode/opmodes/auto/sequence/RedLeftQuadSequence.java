package org.firstinspires.ftc.teamcode.opmodes.auto.sequence;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController;

public class RedLeftQuadSequence extends Sequence {

    public RedLeftQuadSequence(HardwareMap hwMap, Telemetry tel){
        super(hwMap, tel);
    }

    @Override
    public void init() {
        super.init();

        // do some more stuff
    }

    @Override
    public boolean execute() throws InterruptedException {
        if (!super.execute()) {
            return false;
        }

        return true;
    }
}
