package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;

@Autonomous(name="BlueRightAuto", group="Iterative Opmode", preselectTeleOp="BlueTele")
public class BlueRightAuto extends Auto {

    @Override
    public void init(){
        alliance = FieldConstants.Alliance.Blue;
        side = FieldConstants.Side.Right;
        sequenceName = makeSequenceName(alliance, side);

        super.init();
    }
}
