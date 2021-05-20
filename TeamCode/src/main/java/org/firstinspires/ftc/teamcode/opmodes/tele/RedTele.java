package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.auto.Auto;
import org.firstinspires.ftc.teamcode.util.Sleep;
import org.firstinspires.ftc.teamcode.util.TrajectoryHelper;

import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.Alliance;
import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField;
import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField.GoalShotPosTele;
import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField.IntakePos;
import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField.PowerShootingPos;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.RPMGoal;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.RPMPowerShotAuto;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.Red.GoalShotAngle;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.Red.PowerShotAbsoluteAngles;
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

        shooter.spinUp(RPMGoal);
        intake.stopWheels();

        //TODO: fix opmode timeout. make trajectory async
        drive.followTrajectory(TrajectoryHelper.buildCustomSpeedLinearTrajectory(drive, GoalShotPosTele, GoalShotAngle, TeleTrajectorySpeed));
        drive.turnAbsolute(Math.toRadians(GoalShotAngle), 0.95);
        sleep(250);
        intake.stopIntake(false);
        shooter.shoot(3, RPMGoal, true);
    }

    @Override
    protected synchronized void powerShot() {
        drive.stop();
        rearIntake.stop();

        double sleepDelay = 200;
        shooter.spinUp(RPMPowerShotAuto);
        intake.stopWheels();
        drive.followTrajectory(TrajectoryHelper.buildCustomSpeedLinearTrajectory(drive, PowerShootingPos, PowerShotAbsoluteAngles[0], TeleTrajectorySpeed));
        intake.stopIntake(false);

        sleep(sleepDelay);
        drive.turnAbsolute(Math.toRadians(PowerShotAbsoluteAngles[0]), 0.65);
        sleep(sleepDelay);

        shooter.powerShot(RPMPowerShotAuto);
        sleep(sleepDelay);
        for (int i = 1; i < 3; i++) {
            drive.turnAbsolute(Math.toRadians(PowerShotAbsoluteAngles[i]), 0.65);
            sleep(sleepDelay);
            shooter.powerShot(RPMPowerShotAuto);
            sleep(sleepDelay);
        }

        sleep(50); //buffer
        shooter.stop();
    }

    @Override
    protected synchronized void  manualShot() {
        manualShoot = true;
        intake.stopIntake(false);

        shooter.shoot(3, RPMGoal, true);
        while (shooter.shootingState) {
            Sleep.sleep(10);
        }

        manualShoot = false;
    }

    @Override
    protected synchronized void autoIntake() {
        drive.followTrajectory(TrajectoryHelper.buildCustomSpeedLinearTrajectory(drive, IntakePos,0, TeleTrajectorySpeed));
    }

    protected void localizeWithCorner() {
        drive.setPoseEstimate(new Pose2d(RedField.LocalizePos, 0));
    }
}
