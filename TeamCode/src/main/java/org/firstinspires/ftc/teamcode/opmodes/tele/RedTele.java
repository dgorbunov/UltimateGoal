package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.auto.Auto;
import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;
import org.firstinspires.ftc.teamcode.opmodes.auto.sequence.TrajectoryHelper;

import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField.GoalShotPos;
import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField.PowerShotPos;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.PowerShotDelay;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.RPMGoal;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.RPMPowerShot;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.Red;
import static org.firstinspires.ftc.teamcode.util.Sleep.sleep;

@TeleOp(name="RedTele", group="Iterative Opmode")
public class RedTele extends Tele {

    RedTele() {
        super();
        Auto.alliance = FieldConstants.Alliance.Red;
    }

    @Override
    protected void autoShoot() {
        autoShoot = true;
        if (drive.getPoseEstimate().getY() < Red.AutoShootLine) {
            //TODO: try turn and linear move
            //TODO: try switching to async
//            shooter.spinUp(RPMGoal);
            drive.turn(GoalShotPos.getHeading());
            drive.followTrajectoryAsync(TrajectoryHelper.buildAutoShootTrajectory(drive, GoalShotPos, 20));
            new Thread(this::waitingToShoot).start();
        }
        else {
//            shooter.spinUp(RPMPowerShot);
//            drive.turn(Math.toRadians(Red.PowerShotInitialAngle));
            drive.followTrajectory(TrajectoryHelper.buildAutoShootTrajectory(drive, PowerShotPos, 20));

            //TODO: TEST THIS, finish powershot! try turn
            Pose2d targetPose = new Pose2d(
                    drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(),
                    drive.getPoseEstimate().getHeading() + Math.toRadians(Red.PowerShotAngleIncrement));
            drive.followTrajectory(TrajectoryHelper.buildAutoShootTrajectory(drive, targetPose, 20));

            for (int i = 0; i < 3; i++) {
                shooter.powerShot(RPMPowerShot);
                drive.turn(Math.toRadians(Red.PowerShotAngleIncrement));
                sleep(PowerShotDelay);
            }

            shooter.stop();
        }
    }

    public void autoShootBlocking() {
        autoShoot = true;
        if (drive.getPoseEstimate().getY() < Red.AutoShootLine) {
            //TODO: try turn and linear move
            //TODO: try switching to async
//            shooter.spinUp(RPMGoal);
            drive.turn(GoalShotPos.getHeading());
            drive.followTrajectory(TrajectoryHelper.buildAutoShootTrajectory(drive, GoalShotPos, 20));
            shooter.shoot(3, RPMGoal);
        }
        else {
//            shooter.spinUp(RPMPowerShot);
//            drive.turn(Math.toRadians(Red.PowerShotInitialAngle));
            drive.followTrajectory(TrajectoryHelper.buildAutoShootTrajectory(drive, PowerShotPos, 20));

            //TODO: TEST THIS, finish powershot! try turn
            Pose2d targetPose = new Pose2d(
                    drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(),
                    drive.getPoseEstimate().getHeading() + Math.toRadians(Red.PowerShotAngleIncrement));
            drive.followTrajectory(TrajectoryHelper.buildAutoShootTrajectory(drive, targetPose, 20));

            for (int i = 0; i < 3; i++) {
                shooter.powerShot(RPMPowerShot);
                drive.turn(Math.toRadians(Red.PowerShotAngleIncrement));
                sleep(PowerShotDelay);
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
        manShoot = true;
        if (drive.getPoseEstimate().getY() < Red.AutoShootLine) {
            shooter.shoot(3, RPMGoal);
        }
        else {
            powerShotCt++;
            shooter.powerShot(RPMPowerShot);
            if (powerShotCt >= 3) {
                shooter.stop();
                powerShotCt = 0;
            }
        }
    }

    private void waitingToShoot() {
        while (drive.isBusy()) {
            //do nothing
        }
        shooter.shoot(3, RPMGoal);
        autoShoot = false;
    }

}
