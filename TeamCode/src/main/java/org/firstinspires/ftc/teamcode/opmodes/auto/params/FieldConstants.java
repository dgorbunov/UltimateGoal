package org.firstinspires.ftc.teamcode.opmodes.auto.params;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class FieldConstants {

    /**
     * Field/Autonomous Constants
     */

    public static final String RedAlliance = "Red";
    public static final String BlueAlliance = "Blue";
    public static final String LeftSide = "Left";
    public static final String RightSide = "Right";

    public enum Alliance {
        Red, Blue
    }

    public enum Side {
        Left, Right
    }

    public static final String AllTrajectories = "all";

    public static final String NoRings = "None";
    public static final String SingleRing = "Single";
    public static final String QuadRing = "Quad";
    public static final String Camera = "camera";
    public static final String Shooter = "shooter";
    public static final String Intake = "intake";
    public static final String VertIntake = "verticalIntake";
    public static final String Wobble = "wobble";
    public static final String Drive = "drive";
    public static final String Hub = "hub";
    public static final String Bumper = "bumper";

    private static final double TapeWidth  = 1.75;

    //Robot Width: 17.95 in (456mm)
    //Robot Length: 17 in (432mm)

    @Config
    public static class RedField {
        public static final Vector2d TargetZoneA = new Vector2d(-7,-61);
        public static final Vector2d TargetZoneB = new Vector2d(17,-38);
        public static final Vector2d TargetZoneC = new Vector2d(42,-61);
        public static final Vector2d RingPos = new Vector2d(-24, -36.5);
        public static final Pose2d GoalShotPos = new Pose2d(-2, -39, Math.toRadians(0));
        public static final Pose2d PowerShotPos = new Pose2d(-2, -12, Math.toRadians(9));
        public static final Vector2d IntakePos = new Vector2d(RingPos.getX(), RingPos.getY());
        public static final Vector2d IntakeFourPos = new Vector2d(RingPos.getX() + 4, RingPos.getY());
        public static final Vector2d LeftWobblePos = new Vector2d(-37.5, -23.00);
        public static final Vector2d LeftWobbleIntermediate = new Vector2d(LeftWobblePos.getX() + 8, LeftWobblePos.getY());
        public static final Vector2d RightWobblePos = new Vector2d(-39, RedRight.StartingPos.getY());
        public static final Vector2d RightWobbleIntermediate = new Vector2d(RightWobblePos.getX() + 10, RightWobblePos.getY());
        public static final Vector2d EndingPosition = new Vector2d (10, -30);
        public static final Vector2d EndingPositionFour = new Vector2d (10, TargetZoneC.getY());
        public static final double WobbleXOffset = 10;
        public static final double WobbleBackupDistance = 15;
    }

    @Config
    public static class RedLeft {
        public static final Vector2d StartingPos = new Vector2d(-63, -24 - TapeWidth);
        public static final Vector2d LaunchLine = new Vector2d(12 - TapeWidth, StartingPos.getY());
        public static final Vector2d IntermediatePos = new Vector2d(RedField.RingPos.getX(), RedField.RingPos.getY() + 16);
    }

    @Config
    public static class RedRight {
        public static final Vector2d StartingPos = new Vector2d(-63, -46.20);
        public static final Vector2d LaunchLine = new Vector2d(12 - TapeWidth, StartingPos.getY());
        public static final Vector2d IntermediatePos = new Vector2d(RedField.RingPos.getX() + 6, RedField.RingPos.getY() - 19);
    }

    @Config
    public static class BlueField {

    }

    @Config
    public static class BlueLeft {
        public static final Vector2d StartingPos = new Vector2d(-63, 24 + TapeWidth);
    }

    @Config
    public static class BlueRight {
        public static final Vector2d StartingPos = new Vector2d(-63, 48 + TapeWidth);
    }

}
