package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;

import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.RPMGoal;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.RPMPowerShot;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.Red;

@TeleOp(name="RedTele", group="Iterative Opmode")
public class RedTele extends Tele {
    public RedTele() {
        super();
    }

    @Override
    protected void autoShoot() {
        if (drive.getPoseEstimate().getY() < Red.AutoShootLine) {
            shooter.spinUp(RPMGoal);
            Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(FieldConstants.RedField.GoalShotPos, 0))
                    .build();
            drive.followTrajectory(trajectory);
            shooter.shoot(3);
        }
        else {
            shooter.spinUp(RPMPowerShot);
            Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(FieldConstants.RedField.PowerShotPos, Math.toRadians(Red.PowerShotInitialAngle)))
                    .build();
            drive.followTrajectory(trajectory);
            for (int i = 0; i < 3; i++) {
                shooter.powerShot(RPMPowerShot);
                drive.turn(Math.toRadians(Red.PowerShotAngleIncrement));
            }

            shooter.stop();
        }

    }

    @Override
    protected void powerShot() {
        for (int i = 0; i < 3; i++) {
            shooter.powerShot(RPMPowerShot);
            if (i < 2) drive.turn(Math.toRadians(Red.PowerShotAngleIncrement));
        }

        shooter.stop();
    }
}
