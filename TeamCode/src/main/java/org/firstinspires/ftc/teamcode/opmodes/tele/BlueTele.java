package org.firstinspires.ftc.teamcode.opmodes.tele;

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

}
