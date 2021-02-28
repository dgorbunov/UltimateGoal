package org.firstinspires.ftc.teamcode.opmodes.auto.params;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class FieldConstants {

    /**
     * Field/Autonomous Constants
     */

    public static final String RedAlliance = "Red";
    public static final String BlueAlliance = "Blue";
    public static final String LeftSide = "Left";
    public static final String RightSide = "Right";

    public enum Alliance { Red, Blue }
    public enum Side { Left, Right }

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

    //Robot Dimensions: 18 x 18 in (approx.)

    @Config
    public static class RedField {
        public static final Vector2d TargetZoneA = new Vector2d(12,-61);
        public static final Vector2d TargetZoneB = new Vector2d(36,-37);
        public static final Vector2d TargetZoneC = new Vector2d(60,-61);
        public static final Vector2d RingPos = new Vector2d(-24, -36.5);
        public static final Vector2d GoalShotPos = new Vector2d(-2, -39);
        public static final Vector2d PowerShotPos = new Vector2d(-2, -17);
        public static final Vector2d IntakeOnePos = new Vector2d(RingPos.getX() + 2, RingPos.getY());
        public static final Vector2d IntakeFourPos = new Vector2d(RingPos.getX() + 4, RingPos.getY());
        public static final Vector2d LeftWobblePos = new Vector2d(-37.25, -22);
        public static final Vector2d LeftWobbleIntermediate = new Vector2d(LeftWobblePos.getX() + 8, LeftWobblePos.getY());
        public static final Vector2d RightWobblePos = new Vector2d(-37.25, -45);
        public static final Vector2d RightWobbleIntermediate = new Vector2d(RightWobblePos.getX() + 10, RightWobblePos.getY());
        public static final Vector2d EndingPosition = new Vector2d (10, -30);
        public static final Vector2d EndingPositionFour = new Vector2d (10, TargetZoneC.getY());
        public static final double SideWobbleXOffset = 0;
        public static final double SideWobbleYOffset = 9;
        public static final double FrontWobbleXOffset = -20;
        public static final double FrontWobbleYOffset = 0;
    }

    @Config
    public static class RedLeft {
        public static final Vector2d StartingPos = new Vector2d(-61.75, -26.50);
        public static final Vector2d LaunchLine = new Vector2d(12 - TapeWidth, StartingPos.getY());
        public static final Vector2d IntermediatePos = new Vector2d(RedField.RingPos.getX(), RedField.RingPos.getY() + 16);
    }

    @Config
    public static class RedRight {
        public static final Vector2d StartingPos = new Vector2d(-61.75, -50.50); //offset from center beam is 8mm = 0.3 in, tape offset from mat is +- 2.2 in
        public static final Vector2d LaunchLine = new Vector2d(12 - TapeWidth, StartingPos.getY());
        public static final Vector2d IntermediatePos = new Vector2d(RedField.RingPos.getX() + 6, RedField.RingPos.getY() - 19);
        public static final Vector2d PowerShotIntermediatePos = new Vector2d(RedField.PowerShotPos.getX(), -52);
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
