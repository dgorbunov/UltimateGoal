package org.firstinspires.ftc.teamcode.opmodes.tele.params;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;

@Config
public class MechConstants {

    public static Pose2d TeleStartingPose = new Pose2d(FieldConstants.RedLeft.StartingPos, 0);
    /**
     * Set by Sequence
     * @see org.firstinspires.ftc.teamcode.opmodes.auto.sequence.Sequence#stop()
     */

    public static double DriveFullPower = 0.90;
    public static double DriveSlowPower = 0.16;

    public static double RPMGoal = 3750;
    public static double RPMAuto = 3775;
    @Deprecated
    public static double RPMGoalFromStack = 3600;
    public static double RPMPowerShot = 3350; //~3300-3400
    public static double TeleTrajectorySpeed = 0.95;

    @Config
    public static class Red {
        public static double AutoShootLine = -24.5;
        public static double GoalShotAngle = -11.5;
        public static double[] PowerShotAbsoluteAngles = {3,-5,-10}; //prev: {5,-3.5,-9.5}
    }

    @Config
    public static class Blue {

    }

}
