package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;

@Autonomous(name="RedLeftAuto", group="Iterative Opmode", preselectTeleOp="RedTele")
public class RedLeftAuto extends Auto {

    @Override
    public void init(){
        alliance = FieldConstants.Alliance.Red;
        side = FieldConstants.Side.Left;
        sequenceName = makeSequenceName(alliance, side);

        super.init();
    }
}
