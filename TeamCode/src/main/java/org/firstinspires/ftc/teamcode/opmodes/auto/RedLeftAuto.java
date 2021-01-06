package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;

@Autonomous(name="RedLeftAuto", group="Iterative Opmode", preselectTeleOp="RedTele")
@Config //for FTCDash
public class RedLeftAuto extends Auto {

    @Override
    public void init(){
        sequenceName = makeSequenceName(FieldConstants.RedAlliance, FieldConstants.LeftSide);
        super.init();
    }
}
