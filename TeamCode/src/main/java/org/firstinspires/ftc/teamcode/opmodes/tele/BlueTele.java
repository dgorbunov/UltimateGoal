package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;
import org.firstinspires.ftc.teamcode.opmodes.tele.base.Tele;

@TeleOp(name="RedTele", group="Iterative Opmode")
public class BlueTele extends Tele {
    public BlueTele(){
        super();
        alliance = FieldConstants.RedAlliance;
    }

    @Override
    protected void autoShoot() {

    }
}
