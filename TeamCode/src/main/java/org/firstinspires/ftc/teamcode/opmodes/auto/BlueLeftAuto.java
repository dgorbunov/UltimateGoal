package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BlueLeftAuto", group="Iterative Opmode")
@Config //for FTCDash
public class BlueLeftAuto extends FullAuto {

    @Override
    public void init(){
        sequenceName = makeSequenceName(FieldConstants.BlueAlliance, FieldConstants.LeftSide);
        super.init();
    }
}
