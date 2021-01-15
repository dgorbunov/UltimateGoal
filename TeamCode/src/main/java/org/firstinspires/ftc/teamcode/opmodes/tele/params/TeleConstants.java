package org.firstinspires.ftc.teamcode.opmodes.tele.params;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class TeleConstants {

    /**
     * Set by Sequence
     * @see org.firstinspires.ftc.teamcode.opmodes.auto.sequence.Sequence#stop()
     */
    public static Pose2d StartingPose = new Pose2d(0,0,0);

    public static double DriveFullPower = 0.92;
    public static double DriveSlowPower = 0.28;
}
