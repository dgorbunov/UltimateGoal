package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.OpModeBase;
import org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants;

@TeleOp(name="ShooterPIDTuner", group="Iterative Opmode")
@Config
public class ShooterPIDTuner extends OpModeBase {

    //FIRST Motor Velocity PID Tuning Guide:
    //https://docs.google.com/document/u/2/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/mobilebasic

    public static double kP = 2;
    public static double kI = 0.2;
    public static double kD = 6.5;
    public static double F = 11.7;

    @Override
    public void init() {
        super.init();
        shooter.setVelocityPIDFCoefficients(kP, kI, kD, F);
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();

        shootButton.runOnce(
                gameMap.Shoot(),
                () -> shooter.shootAsync(3, MechConstants.RPMGoal));

        shootManButton.runOnce(
                gameMap.PowerShot(),
                () -> shooter.spinUp(MechConstants.RPMGoal));

        powerShotButton.runOnce(
                gameMap.AutoIntake(),
                () -> shooter.stop());

        drive.putPacketData("shooter rpm", shooter.getCurrentRPM());
        drive.putPacketData("target rpm", shooter.getTargetRPM());
        drive.update();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
