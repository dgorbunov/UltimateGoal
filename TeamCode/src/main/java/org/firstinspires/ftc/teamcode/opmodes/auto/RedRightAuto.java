package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="RedRightAuto", group="Iterative Opmode")
@Config //for FTCDash
public class RedRightAuto extends FullAuto {
    RedRightAuto() {
        SetSequenceName(makeSequenceName(Constants.RedAlliance, Constants.RightSide));
    }
}
