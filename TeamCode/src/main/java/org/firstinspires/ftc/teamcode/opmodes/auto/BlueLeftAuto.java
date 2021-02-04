package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;

@Autonomous(name="BlueLeftAuto", group="Iterative Opmode", preselectTeleOp="BlueTele")
public class BlueLeftAuto extends Auto {

    @Override
    public void init(){
        alliance = FieldConstants.Alliance.Blue;
        side = FieldConstants.Side.Left;
        sequenceName = makeSequenceName(alliance, side);

        super.init();
    }
}
