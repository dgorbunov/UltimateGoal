package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;

@Autonomous(name="BlueLeftAuto", group="Iterative Opmode", preselectTeleOp="BlueTele")
@Config //for FTCDash
public class BlueLeftAuto extends Auto {

    @Override
    public void init(){
        sequenceName = makeSequenceName(FieldConstants.BlueAlliance, FieldConstants.LeftSide);
        super.init();
    }
}
