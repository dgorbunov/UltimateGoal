package org.firstinspires.ftc.teamcode.opmodes.tele.params;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class MechConstants {

    /**
     * Set by Sequence
     * @see org.firstinspires.ftc.teamcode.opmodes.auto.sequence.Sequence#stop()
     * TODO: write to a file
     */
    public static Pose2d StartingPose = new Pose2d(0.50,-36,0);

    public static double DriveFullPower = 0.80;
    public static double DriveSlowPower = 0.20;

    public static double RPMGoal = 3650;
    public static double RPMPowerShot = 3500;
    public static double PowerShotDelay = 600;

    @Config
    public static class Red {
        public static double PowerShotInitialAngle = 9;
        public static double PowerShotAngleIncrement = -6;
        public static double AutoShootLine = -24.5;
    }

    @Config
    public static class Blue {

    }



}
