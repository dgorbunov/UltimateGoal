package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="RedLeftAuto", group="Iterative Opmode")
@Config //for FTCDash
public class RedLeftAuto extends FullAuto {
    RedLeftAuto() {
        SetSequenceName(makeSequenceName(FieldConstants.RedAlliance, FieldConstants.LeftSide));
    }
}
