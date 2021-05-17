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
    public static final String PacketManager = "packetManager";
    public static final String Drive = "drive";
    public static final String Hub = "hub";
    public static final String Bumper = "bumper";

    private static final double TapeWidth  = 1.75;

    //Robot Dimensions: 18 x 18 in (approx.)

    @Config
    public static class RedField {
        public static final Vector2d TargetZoneA = new Vector2d(12,-61);
        public static final Vector2d TargetZoneB = new Vector2d(36,-37);
        public static final Vector2d TargetZoneC = new Vector2d(58.5,-61); //really x=60
        public static final Vector2d RingPos = new Vector2d(-24, -36.5);
        public static final Vector2d GoalShotPos = new Vector2d(-4, -41);
        public static final Vector2d PowerShotStrafePos = new Vector2d(-4, -8);
        public static final Vector2d IntakeOnePos = RingPos;
        public static final Vector2d IntakeFourPos = new Vector2d(RingPos.getX() + 12, RingPos.getY());
        public static final Vector2d LeftWobblePos = new Vector2d(-36, -20);
        public static final Vector2d LeftWobbleIntermediate = new Vector2d(LeftWobblePos.getX() + 7, LeftWobblePos.getY());
        public static final Vector2d RightWobblePos = new Vector2d(-35.5, -49);
        public static final Vector2d RightWobbleIntermediate = new Vector2d(RightWobblePos.getX() + 7, RightWobblePos.getY());
        public static final Vector2d EndingPosition = new Vector2d (10, -30);
        public static final Vector2d EndingPositionFour = new Vector2d (12.5, TargetZoneC.getY());
        public static final Vector2d TopCornerPos = new Vector2d(62.00, -62.50);
        public static final Vector2d LocalizePos = new Vector2d(14.75, -40.25);
        public static final Vector2d IntakePos = new Vector2d(-4, -24);
        public static final Vector2d GoalShotPosTele = IntakePos;
        public static final Vector2d GoalPos = new Vector2d(72, -36);
        public static final Vector2d MiddlePowerShotPos = new Vector2d (72, -12);
        public static final Vector2d PowerShootingPos = new Vector2d(RingPos.getX() - 12, RingPos.getY());
        public static double PowerShotOffset = 7.5;
        public static final double SideWobbleXOffset = 0;
        public static final double SideWobbleYOffset = 6;
        public static final double FrontWobbleXOffset = -15;
        public static final double FrontWobbleYOffset = 3;
        public static final double PowerShotY1 = -10;
        public static final double PowerShotY2 = -15;
        public static final double PowerShotY3 = -20;
    }

    @Config
    public static class RedLeft {
        public static final Vector2d StartingPos = new Vector2d(-63.00, -25);
        public static final Vector2d LaunchLine = new Vector2d(12 - TapeWidth, StartingPos.getY());
        public static final Vector2d IntermediatePos = new Vector2d(RedField.RingPos.getX(), RedField.RingPos.getY() + 16);
    }

    @Config
    public static class RedRight {
        public static final Vector2d StartingPos = new Vector2d(-61.75, -50.50); //offset from center beam is 8mm = 0.3 in, tape offset from mat is +- 2.2 in
        public static final Vector2d LaunchLine = new Vector2d(12 - TapeWidth, StartingPos.getY());
        public static final Vector2d IntermediatePos = new Vector2d(RedField.RingPos.getX() + 6, RedField.RingPos.getY() - 19);
        public static final Vector2d PowerShotIntermediatePos = new Vector2d(RedField.PowerShootingPos.getX(), -52);
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
