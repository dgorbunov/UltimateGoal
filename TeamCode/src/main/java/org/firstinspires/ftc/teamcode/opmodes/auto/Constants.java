package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Constants {

    /**
     * Auto Global Constants
     */

    public static final String RedAlliance = "Red";
    public static final String BlueAlliance = "Blue";
    public static final String LeftSide = "Left";
    public static final String RightSide = "Right";

    public static final String NoRings = "None";
    public static final String SingleRing = "Single";
    public static final String QuadRing = "Quad";
    public static final String Camera = "camera";
    public static final String Shooter = "shooter";
    public static final String Intake = "intake";
    public static final String Wobble = "wobble";
    public static final String Drive = "drive";
    public static final String Hub = "hub";

    private static final double tapeWidth  = 1.75;


    //TODO: Swap with vectors
    public static class RedAlliance {
        public static final Pose2d targetzoneA = new Pose2d(0,0,0);
        public static final Pose2d targetzoneB = new Pose2d(0,0,0);
        public static final Pose2d targetzoneC = new Pose2d(0,0,0);
    }

    public static class RedLeft {
        public static final Pose2d startingPose = new Pose2d(-63, -24 - tapeWidth, Math.toRadians(180));

    }

    public static class RedRight {
        public static final Pose2d startingPose = new Pose2d(-63, -48 - tapeWidth, Math.toRadians(180));
        public static final double StartingPosY = -25.75;
    }

    public static class BlueLeft {
        public static final Pose2d startingPose = new Pose2d(-63, 24 + tapeWidth, Math.toRadians(180));
    }

    public static class BlueRight {
        public static final Pose2d startingPose = new Pose2d(-63, 48 + tapeWidth, Math.toRadians(180));
    }

}
