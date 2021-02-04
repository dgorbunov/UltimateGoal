package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;

@Autonomous(name="RedRightAuto", group="Iterative Opmode", preselectTeleOp="RedTele")
public class RedRightAuto extends Auto {

    @Override
    public void init(){
        alliance = FieldConstants.Alliance.Red;
        side = FieldConstants.Side.Right;
        sequenceName = makeSequenceName(alliance, side);

        super.init();
    }
}
