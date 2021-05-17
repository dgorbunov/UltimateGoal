package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.OpModeBase;
import org.firstinspires.ftc.teamcode.opmodes.auto.Auto;
import org.firstinspires.ftc.teamcode.opmodes.tele.params.GamepadMappings;
import org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants;
import org.firstinspires.ftc.teamcode.robot.camera.algorithms.VerticalRingDetector;
import org.firstinspires.ftc.teamcode.robot.systems.IntakeController;
import org.firstinspires.ftc.teamcode.robot.systems.ShooterController;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField.GoalPos;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.DriveFullPower;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.DriveSlowPower;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.RPMGoal;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.TurretOffsetAdjustment;

@TeleOp(name="Tele", group="Iterative Opmode")
@Disabled
public abstract class Tele extends OpModeBase {

    public static volatile GamepadMappings.DriverMode DriverMode = GamepadMappings.DriverMode.TwoDrivers;

    protected boolean manualShoot = false;
    private double loopTime;
    private double initTime;

    public Tele() {
        super();
    }

    @Override
    public void init() {
        OPMODE_TYPE = OPMODE.Tele;
        super.init();
        initTime = systemClock.seconds();

        drive.setPoseEstimate(MechConstants.TeleStartingPose);
        shooter.spinUp(RPMGoal);
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();

        loopTime = systemClock.seconds();
        double matchTime = loopTime - initTime;

//        if (shooter.shootingState) {
//            //whenever shooting, drive slow
//            drive.driveFieldCentric(gamepad1, DriveSlowPower, Auto.getAlliance());
//            driveModeButton.resetToggle();

//        } else {
            shootButton.runOnce(gameMap.Shoot(), () -> shooter.shoot(1, MechConstants.RPMGoal, false), () -> intake.gateLift());
            //TODO: FIX OPMODE STUCK LOOP TIMEOUT, USE ITERATIVE OP MODE
            if (matchTime > 85) {
                driveModeButton.toggleLoop(
                        gameMap.DriveMode(),
                        () -> drive.driveFieldCentric(gamepad1, 1.0, Auto.getAlliance()),
                        () -> drive.driveFieldCentric(gamepad1, DriveSlowPower, Auto.getAlliance())
                );
            } else {
                driveModeButton.toggleLoop(
                        gameMap.DriveMode(),
                        () -> drive.driveFieldCentric(gamepad1, DriveFullPower, Auto.getAlliance()),
                        () -> drive.driveFieldCentric(gamepad1, DriveSlowPower, Auto.getAlliance())
                );
            }
//        }

        drive.update();

        incrementLeftButton.runOnceBlocking(
                gameMap.IncrementLeft(),
                () -> shooter.incrementTurretOffset(-TurretOffsetAdjustment));

        incrementRightButton.runOnceBlocking(
                gameMap.IncrementRight(),
                () -> shooter.incrementTurretOffset(TurretOffsetAdjustment));

        intakeButton.toggle(
                gameMap.Intake(),
                () -> intake.run(FORWARD),
                () -> intake.run(REVERSE),
                () -> intake.stopIntake());

        resetIntakeCounterButton.runOnce(
                gameMap.ResetIntakeCounter(),
                () -> IntakeController.numRings.set(0));

        wobbleButton.toggle(
                gameMap.Wobble(),
                () -> wobble.dropTele(),
                () -> wobble.pickupTele());

        wobbleDeliverButton.toggle(
                gameMap.WobbleDeliver(),
                () -> wobble.release());

        powerShotButton.runOnceBlocking(
                gameMap.PowerShot(),
                this::powerShot);

        autoIntakeButton.runOnceBlocking(
                gameMap.AutoIntake(),
                this::autoIntake);

        localizeButton.runOnce(
                gameMap.Localize(),
                this::localizeWithCorner
        );

        stopIntakeButton.runOnce(
                gameMap.StopAllIntakes(),
                () -> intake.stopIntake(),
                () -> rearIntake.stop(),
                () -> intakeButton.resetToggle()
        );

        if (gameMap.Intake()) intake.gateLower();

        shooter.updateTurret(drive.getPoseEstimate(), GoalPos);

        drive.putPacketData("shooter RPM", shooter.getCurrentRPM());
        drive.putPacketData("target RPM", shooter.getTargetRPM());
        drive.putPacketData("loop time", Math.round(systemClock.seconds() * 1000 - loopTime  * 1000));
        telemetryd.addData("Rings Intaked", IntakeController.getNumRings());
        telemetryd.addLine("<strong>Turret Offset: </strong>" + ShooterController.TURRET_OFFSET);
        telemetryd.addData("Rings: Aspect Ratio", VerticalRingDetector.getAspectRatio());
        telemetryd.addData("Rings: Width", VerticalRingDetector.getRingWidth());
        telemetryd.addLine("<strong>Match time: </strong>" + Math.round(matchTime));
        telemetryd.addLine("<strong>Using: </strong>" + hub.getFormattedCurrentDraw());
        telemetryd.addLine("<strong>Loop Time: </strong>" + Math.round(systemClock.seconds() * 1000 - loopTime  * 1000) + " ms");

    }

    protected abstract void autoShot();
    protected abstract void powerShot();
    protected abstract void manualShot();
    protected abstract void autoIntake();
    protected abstract void localizeWithCorner();

    @Override
    public void stop() {
        super.stop();
    }

}
