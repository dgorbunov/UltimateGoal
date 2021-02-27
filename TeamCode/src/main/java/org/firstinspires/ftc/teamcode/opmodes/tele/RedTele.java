package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.auto.Auto;
import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;
import org.firstinspires.ftc.teamcode.opmodes.auto.sequence.TrajectoryHelper;
import org.firstinspires.ftc.teamcode.util.Sleep;

import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField.GoalShotPos;
import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField.PowerShotPos;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.PowerShotDelay;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.RPMGoal;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.RPMPowerShot;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.Red;
import static org.firstinspires.ftc.teamcode.util.Sleep.sleep;

@TeleOp(name="RedTele", group="Iterative Opmode")
public class RedTele extends Tele {

    public RedTele() {
        super();
        Auto.alliance = FieldConstants.Alliance.Red;
    }

    @Override
    protected void autoShoot() {
        autoShoot = true;
        drive.stop();
        intake.stopIntake();

        if (drive.getPoseEstimate().getY() < Red.AutoShootLine) {
            drive.turn(GoalShotPos.getHeading());
            drive.followTrajectory(TrajectoryHelper.buildAutoShootTrajectory(drive, GoalShotPos, 15));
            shooter.shootAsync(3, RPMGoal);
        }
        else {
            drive.followTrajectory(TrajectoryHelper.buildAutoShootTrajectory(drive, PowerShotPos, 15));

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
        drive.stop();
        intake.stopIntake();

        for (int i = 0; i < 3; i++) {
            shooter.powerShot(RPMPowerShot);
            if (i < 2) drive.turn(Math.toRadians(Red.PowerShotAngleIncrement));
        }

        shooter.stop();

        autoShoot = false;
    }

    @Override
    protected void manualShoot() {
        manualShoot = true;
        intake.stopIntake();
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
        while (shooter.shootingState){
            Sleep.sleep(10);
        }
        manualShoot = false;
    }

    protected void localize() {
        drive.setPoseEstimate(new Pose2d(63, -63, 0)); //front right corner
    }



}
