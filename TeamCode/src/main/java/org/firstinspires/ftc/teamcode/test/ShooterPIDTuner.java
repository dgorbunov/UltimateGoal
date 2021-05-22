package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.OpModeBase;
import org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants;

import static org.firstinspires.ftc.teamcode.robot.systems.ShooterController.MAX_VEL;

@TeleOp(name="ShooterPIDTuner", group="Iterative Opmode")
@Config
public class ShooterPIDTuner extends OpModeBase {

    //FIRST Motor Velocity PID Tuning Guide:
    //https://docs.google.com/document/u/2/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/mobilebasic

    public static final double TICKS_PER_REV = 28.0;
    public static volatile double kP = 6;
    public static volatile double kI = 0.2;
    public static volatile double kD = 6.5;
    public static volatile double F = 11.7;
    public static volatile double kP2 = 5;
    public static volatile double kI2 = 0.3;
    public static volatile double kD2 = 0;
    public static volatile double F2 = 11.7;
    public static volatile boolean useSecondSetInLoop = true;
    public static double shootRPM = MechConstants.RPMGoalAuto;
    public static double targetRPM = MechConstants.RPMGoalAuto + 75;

    @Override
    public void init() {
        super.init();
        shooter.setVelocityPIDFCoefficients(kP, kI, kD, F);
    }

    @Override
    public void init_loop() {
        super.init_loop();
        shooter.spinUp(shootRPM);
    }

    private void calculatePID() {
        //https://docs.google.com/document/u/2/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/mobilebasic
        MAX_VEL = RPMtoTPS(6000);
        F = 32767 / MAX_VEL;
        kP = 0.1 * F;
        kI = 0.1 * kP;
        kD = 0;
    }

    private double RPMtoTPS(double RPM){
        return RPM * TICKS_PER_REV / 60;
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();

        if (useSecondSetInLoop) shooter.setVelocityPIDFCoefficients(kP2, kI2, kD2, F2);
        else shooter.setVelocityPIDFCoefficients(kP, kI, kD, F);

        shootButton.toggle(gamepad1.y, () -> shooter.spinUp(shootRPM), () -> shooter.spinUp(targetRPM));

        shootManButton.toggle(gamepad1.a, () -> shooter.shoot(1, shootRPM, false));

        wobbleButton.runOnce(gamepad1.b, () -> shooter.spinUp(shootRPM));

        wobbleDeliverButton.runOnce(gamepad1.x, () -> shooter.stopShooter());

        drive.putPacketData("shooter rpm", shooter.getCurrentRPM());
        drive.putPacketData("target rpm", shooter.getTargetRPM());
        drive.update();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
