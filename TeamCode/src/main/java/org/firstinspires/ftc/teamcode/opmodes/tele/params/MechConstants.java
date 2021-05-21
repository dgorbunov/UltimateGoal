package org.firstinspires.ftc.teamcode.opmodes.tele.params;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;

@Config
public class MechConstants {

    public static Pose2d TeleStartingPose = new Pose2d(FieldConstants.RedField.LocalizePos, 0);
    /**
     * This is a default, it is set by Sequence
     * @see org.firstinspires.ftc.teamcode.opmodes.auto.sequence.Sequence#stop()
     */

    public static double DriveFullPower = 0.90;
    public static double DriveSlowPower = 0.16;

    public static double RPMGoal = 3175;
    public static double RPMAuto = 3850;
    public static double RPMPowerShotAuto = 3230;
    public static double TurretOffsetAdjustment = 2;
    public static double TeleTrajectorySpeed = 0.95;

    @Config
    public static class Red {
        public static double GoalShotAngle = -12.0;
        public static double[] PowerShotAbsoluteAngles = {3,-6.5,-11.5}; //prev: {5,-3.5,-9.5}

        public static void incrementGoalShotAngle(double inc) {
            GoalShotAngle += inc;
        }
    }

    @Config
    public static class Blue {

    }

}
