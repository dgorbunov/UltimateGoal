package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.auto.Auto;
import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;
import org.firstinspires.ftc.teamcode.robot.systems.ShooterController;
import org.firstinspires.ftc.teamcode.util.Sleep;
import org.firstinspires.ftc.teamcode.util.TrajectoryHelper;

import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.Alliance;
import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField;
import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField.GoalShotPosTele;
import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField.IntakePos;
import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField.MiddlePowerShotPos;
import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField.PowerShotOffset;
import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField.PowerShotPosAuto;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.RPMGoalAuto;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.RPMPowerShot;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.Red.GoalShotAngle;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.TeleTrajectorySpeed;
import static org.firstinspires.ftc.teamcode.util.Sleep.sleep;

@TeleOp(name="RedTele", group="Iterative Opmode")
public class RedTele extends Tele {

    public RedTele() {
        super();
        Auto.alliance = Alliance.Red;
    }

    @Override
    protected synchronized void autoShot() {
        drive.stop();
        rearIntake.stop();

        shooter.spinUp(RPMGoalAuto);
        intake.stopWheels();

        //TODO: fix opmode timeout. make trajectory async
        drive.followTrajectory(TrajectoryHelper.buildCustomSpeedLinearTrajectory(drive, GoalShotPosTele, GoalShotAngle, TeleTrajectorySpeed));
        drive.turnAbsolute(Math.toRadians(GoalShotAngle), 0.95);
        sleep(250);
        intake.stopIntake(false);
        shooter.shoot(3, RPMGoalAuto, true);
    }

    @Override
    protected synchronized void powerShot() {
        telemetry.addData("Sequence", "powerShot");
        ShooterController shooter = controllers.get(ShooterController.class, FieldConstants.Shooter);
        double sleep = 500;

        drive.followTrajectory(TrajectoryHelper.buildCustomSpeedLinearTrajectory(drive, PowerShotPosAuto, 0, 0.90));

        Vector2d[] targets = {
                new Vector2d(FieldConstants.RedField.MiddlePowerShotPos.getX(), MiddlePowerShotPos.getY() + PowerShotOffset),
                new Vector2d(FieldConstants.RedField.MiddlePowerShotPos.getX(), MiddlePowerShotPos.getY()),
                new Vector2d(FieldConstants.RedField.MiddlePowerShotPos.getX(), MiddlePowerShotPos.getY() - PowerShotOffset),
        };

        double[] speeds = {
                RPMPowerShot,
                RPMPowerShot,
                RPMPowerShot,
        };
        for (int i = 0; i < 3; i++) {
            drive.update();
            shooter.updateTurretAuto(drive.getPoseEstimate(), targets[i]);
            sleep(sleep * 1.5);
            shooter.powerShot(RPMPowerShot);
            shooter.updateTurretAuto(drive.getPoseEstimate(), new Vector2d(drive.getPoseEstimate().getX() + 10, drive.getPoseEstimate().getY() - 5)); //reset
            shooter.spinUp(speeds[i]);
            sleep(sleep);
        }
    }

    @Override
    protected synchronized void  manualShot() {
        manualShoot = true;
        intake.stopIntake(false);

        shooter.shoot(3, RPMGoalAuto, true);
        while (shooter.shootingState) {
            Sleep.sleep(10);
        }

        manualShoot = false;
    }

    @Override
    protected synchronized void autoIntake() {
        drive.stop();
        drive.followTrajectory(TrajectoryHelper.buildCustomSpeedLinearTrajectory(drive, IntakePos,0, TeleTrajectorySpeed));
    }

    protected void localizeWithCorner() {
        drive.setPoseEstimate(new Pose2d(RedField.LocalizePos, 0));
    }
}
