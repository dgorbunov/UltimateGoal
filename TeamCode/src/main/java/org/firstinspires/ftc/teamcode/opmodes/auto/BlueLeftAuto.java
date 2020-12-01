package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BlueLeftAuto", group="Iterative Opmode")
@Config //for FTCDash
public class BlueLeftAuto extends FullAuto {
    BlueLeftAuto() {
        SetSequenceName(makeSequenceName(Constants.BlueAlliance, Constants.LeftSide));
    }
}
