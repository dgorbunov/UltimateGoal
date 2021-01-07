package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="RedTele", group="Iterative Opmode")
public class RedTele extends Tele {
    public RedTele() {
        super();
    }

    @Override
    protected void autoShoot() {
        //drive.autoTakeOver(); //from manual control
        //TODO: need to make sure it's looking at the right target and not a random one
        //TODO: need to make sure it's knows what side it's on
//        drive.setPoseEstimate(camera.getRobotPosition());
//        Trajectory trajectory = drive.trajectoryBuilder(drive.GetCurrentPose())
//                .lineTo(new Pose2d(Field))
//                .build();
//        drive.followTrajectoryAsync(new Trja);
//        drive.waitForIdle();
    }
}
