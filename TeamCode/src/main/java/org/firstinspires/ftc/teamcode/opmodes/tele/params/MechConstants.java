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
    public static double DriveSlowPower = 0.22;

    public static double RPMGoal = 3700;
    @Deprecated
    public static double RPMGoalFromStack = 3600;
    public static double RPMPowerShot = 3250; //~3300-3400
    public static double TeleTrajectorySpeed = 0.90;

    @Config
    public static class Red {
        public static double AutoShootLine = -24.5;
        public static double GoalShotAngle = -9.5;
        public static double[] PowerShotAbsoluteAngles = {4,-3,-9.5}; //prev: {5,-3.5,-9.5}

    }

    @Config
    public static class Blue {

    }

}
