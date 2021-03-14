package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.auto.Auto;
import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;
import org.firstinspires.ftc.teamcode.robot.systems.ShooterController;
import org.firstinspires.ftc.teamcode.util.Sleep;
import org.firstinspires.ftc.teamcode.util.TrajectoryHelper;

import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField.GoalShotPos;
import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField.TopCornerPos;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.RPMGoal;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.RPMPowerShot;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.Red;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.Red.PowerShotAbsoluteAngles;
import static org.firstinspires.ftc.teamcode.util.Sleep.sleep;

@TeleOp(name="RedTele", group="Iterative Opmode")
public class RedTele extends Tele {

    public RedTele() {
        super();
        Auto.alliance = FieldConstants.Alliance.Red;
    }

    @Override
    protected synchronized void autoShot() {
        autoShoot = true;

        drive.stop();
        intake.stopIntake();
        vertIntake.stop();
        sleep(400);

        if (drive.getPoseEstimate().getY() < Red.AutoShootLine) {
            drive.turnAbsolute(0);
            drive.followTrajectory(TrajectoryHelper.buildCustomSpeedLinearTrajectory(drive, GoalShotPos.getX(), GoalShotPos.getY(), 0, 0.80));
            shooter.shootAsync(3, RPMGoal);
        } else {
            powerShot();
        }
        autoShoot = false;
    }

    @Override
    protected synchronized void powerShot() {
        autoShoot = true;
        telemetry.addData("Sequence", "powerShot");

        drive.stop();
        intake.stopIntake();
        vertIntake.stop();
        sleep(400);

        ShooterController shooter = controllers.get(ShooterController.class, FieldConstants.Shooter);
        double sleepDelay = 200;

        sleep(sleepDelay);
        shooter.spinUp(RPMPowerShot);
        drive.turnAbsolute(Math.toRadians(PowerShotAbsoluteAngles[0]));
        sleep(sleepDelay);

        boolean twoRings = false; //hit three powershots with two rings
        if (twoRings) {
            shooter.powerShot(RPMPowerShot);
            drive.turnAbsolute(Math.toRadians(PowerShotAbsoluteAngles[1] + PowerShotAbsoluteAngles[2]));
            shooter.powerShot(RPMPowerShot);
        } else {
            shooter.powerShot(RPMPowerShot);
            sleep(sleepDelay);

            for (int i = 1; i < 3; i++) {
                drive.turnAbsolute(Math.toRadians(PowerShotAbsoluteAngles[i]));
                sleep(sleepDelay);
                shooter.powerShot(RPMPowerShot);
                sleep(sleepDelay);
            }
        }

        sleep(50); //buffer
        shooter.stop();

        autoShoot = false;
    }

    @Override
    protected synchronized void  manualShot() {
        manualShoot = true;

        intake.stopIntake();

        shooter.shoot(3, RPMGoal);
        while (shooter.shootingState) {
            Sleep.sleep(10);
        }

        manualShoot = false;
    }

    protected void localizeWithCorner() {
        drive.setPoseEstimate(new Pose2d(TopCornerPos, 0)); //front right corner
    }
}
