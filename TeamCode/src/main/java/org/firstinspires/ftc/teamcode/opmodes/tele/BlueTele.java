package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.auto.Auto;
import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;

@TeleOp(name="BlueTele", group="Iterative Opmode")
public class BlueTele extends Tele {

    BlueTele(){
        super();
        Auto.alliance = FieldConstants.Alliance.Blue;
    }

    @Override
    protected void autoShoot() {

    }

    @Override
    protected void powerShot() {

    }

    @Override
    protected void manualShoot() {

    }

    @Override
    protected void localize() {
        drive.setPoseEstimate(new Pose2d(61.75, 61.75, 0)); //front left corner
    }

}
