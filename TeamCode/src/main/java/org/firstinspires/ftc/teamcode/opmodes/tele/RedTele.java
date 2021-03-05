package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.auto.Auto;
import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;
import org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants;
import org.firstinspires.ftc.teamcode.util.Sleep;
import org.firstinspires.ftc.teamcode.util.TrajectoryHelper;

import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField.GoalShotPos;
import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField.PowerShotPos;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.RPMGoal;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.RPMPowerShot;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.Red;
import static org.firstinspires.ftc.teamcode.util.Sleep.sleep;
import static org.firstinspires.ftc.teamcode.util.TrajectoryHelper.buildLinearTrajectory;

@TeleOp(name="RedTele", group="Iterative Opmode")
public class RedTele extends Tele {

    public RedTele() {
        super();
        Auto.alliance = FieldConstants.Alliance.Red;
    }

    @Override
    protected void autoShot() {
        autoShoot = true;
        drive.stop();
        intake.stopIntake();

        if (drive.getPoseEstimate().getY() < Red.AutoShootLine) {
            drive.turn(Math.toRadians(0));
            drive.followTrajectory(TrajectoryHelper.buildAutoShootTrajectory(drive, new Pose2d(GoalShotPos, Math.toRadians(0)), 45, 40));
            shooter.shootAsync(3, RPMGoal);
        }
        else {
            drive.followTrajectory(TrajectoryHelper.buildAutoShootTrajectory(drive, new Pose2d(PowerShotPos, Math.toRadians(0)), 45, 40));

            drive.turn(Math.toRadians(-1 * MechConstants.Red.PowerShotAngleIncrement));
            shooter.powerShot(RPMPowerShot);
            for (int i = 0; i < 2; i++) {
                drive.turn(Math.toRadians(MechConstants.Red.PowerShotAngleIncrement));
                shooter.powerShot(RPMPowerShot);
            }
            shooter.stop();
        }
        autoShoot = false;
    }

    @Override
    protected void manualPowerShot() {
        autoShoot = true;
        drive.stop();
        intake.stop();

        shooter.spinUp(RPMPowerShot);
        drive.followTrajectory(buildLinearTrajectory(drive, PowerShotPos.getX(), PowerShotPos.getY(), 0));

        boolean twoRings = false; //hit three powershots with two rings
        if (twoRings) {
            shooter.powerShot(RPMPowerShot);
            drive.turn(Math.toRadians(MechConstants.Red.PowerShotAngleIncrement));
            shooter.powerShot(RPMPowerShot);

        } else {
            drive.turn(Math.toRadians(-1 * MechConstants.Red.PowerShotAngleIncrement));
            shooter.powerShot(RPMPowerShot);

            for (int i = 0; i < 2; i++) {
                drive.turn(Math.toRadians(MechConstants.Red.PowerShotAngleIncrement));
                shooter.powerShot(RPMPowerShot);
            }
        }

        sleep(50); //buffer
        autoShoot = false;
    }

    @Override
    protected void manualShot() {
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

    protected void localizeWithCorner() {
        drive.setPoseEstimate(new Pose2d(61.75, -61.75, 0)); //front right corner
    }

}
