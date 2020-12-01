package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BlueRightAuto", group="Iterative Opmode")
@Config //for FTCDash
public class BlueRightAuto extends FullAuto {
    BlueRightAuto() {
        SetSequenceName(makeSequenceName(FieldConstants.BlueAlliance, FieldConstants.RightSide));
    }
}
