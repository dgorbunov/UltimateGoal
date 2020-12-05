package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="RedLeftAuto", group="Iterative Opmode")
@Config //for FTCDash
public class RedLeftAuto extends FullAuto {

    @Override
    public void init(){
        sequenceName = makeSequenceName(FieldConstants.RedAlliance, FieldConstants.LeftSide);
        super.init();
    }

}
