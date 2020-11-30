package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class Constants {

    /**
     * Auto Global Constants
     */

    public static final String RedAlliance = "Red";
    public static final String BlueAlliance = "Blue";
    public static final String LeftSide = "Left";
    public static final String RightSide = "Right";

    public static final String AllTrajectories = "all";

    public static final String NoRings = "None";
    public static final String SingleRing = "Single";
    public static final String QuadRing = "Quad";
    public static final String Camera = "camera";
    public static final String Shooter = "shooter";
    public static final String Intake = "intake";
    public static final String Wobble = "wobble";
    public static final String Drive = "drive";
    public static final String Hub = "hub";

    private static final double TapeWidth  = 2;

    public static class RedField {
        public static final Vector2d TargetZoneA = new Vector2d(12,-60);
        public static final Vector2d TargetZoneB = new Vector2d(36,-36);
        public static final Vector2d TargetZoneC = new Vector2d(60,-60);
        public static final Vector2d RingPos = new Vector2d(-24.5 + TapeWidth, -36);
    }

    public static class RedLeft {
        public static final Vector2d StartingPos = new Vector2d(-63, -24 - TapeWidth);
        public static final Vector2d ShootingPos = new Vector2d(-6, StartingPos.getY());
        public static final Vector2d LaunchLine = new Vector2d(12 - TapeWidth, StartingPos.getY());
        public static final Vector2d LeftWobblePos = new Vector2d(-39, StartingPos.getY());

    }

    public static class RedRight {
        public static final Vector2d StartingPos = new Vector2d(-63, -48 - TapeWidth);
        public static final Vector2d ShootingPos = new Vector2d(-6, StartingPos.getY());
        public static final Vector2d LaunchLine = new Vector2d(12 - TapeWidth, StartingPos.getY());
        public static final Vector2d RightWobblePos = new Vector2d(-39, StartingPos.getY());
    }

    public static class BlueField {

    }

    public static class BlueLeft {
        public static final Vector2d StartingPos = new Vector2d(-63, 24 + TapeWidth);
    }

    public static class BlueRight {
        public static final Vector2d StartingPos = new Vector2d(-63, 48 + TapeWidth);
    }

}
