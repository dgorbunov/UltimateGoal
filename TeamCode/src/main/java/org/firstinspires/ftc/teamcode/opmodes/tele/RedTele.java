package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;

import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.PowerShotDelay;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.RPMGoal;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.RPMPowerShot;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.Red;
import static org.firstinspires.ftc.teamcode.util.Sleep.sleep;

@TeleOp(name="RedTele", group="Iterative Opmode")
public class RedTele extends Tele {
    private int powerShotCount = 0;

    public RedTele() {
        super();
    }

    @Override
    protected void autoShoot() {
        autoShoot = true;
        if (drive.getPoseEstimate().getY() < Red.AutoShootLine) {
            shooter.spinUp(RPMGoal);
            Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(FieldConstants.RedField.GoalShotPos)
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
                sleep(PowerShotDelay);
                drive.turn(Math.toRadians(Red.PowerShotAngleIncrement));
            }

            shooter.stop();
        }
        autoShoot = false;

    }

    @Override
    protected void powerShot() {
        autoShoot = true;

        for (int i = 0; i < 3; i++) {
            shooter.powerShot(RPMPowerShot);
            if (i < 2) drive.turn(Math.toRadians(Red.PowerShotAngleIncrement));
        }

        shooter.stop();

        autoShoot = false;
    }

    @Override
    protected void manShoot() {
        if (drive.getPoseEstimate().getY() < Red.AutoShootLine) {
            shooter.shoot(3, RPMGoal);
        }
        else {
            powerShotCount++;
            shooter.powerShot(RPMPowerShot);
            if (powerShotCount >= 3) {
                shooter.stop();
                powerShotCount = 0;
            }
        }
    }
}
