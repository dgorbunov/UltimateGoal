package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.OpModeBase;
import org.firstinspires.ftc.teamcode.opmodes.auto.Auto;
import org.firstinspires.ftc.teamcode.opmodes.tele.params.GamepadMappings;
import org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants;
import org.firstinspires.ftc.teamcode.robot.camera.algorithms.VerticalRingDetector;
import org.firstinspires.ftc.teamcode.robot.systems.IntakeController;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.DriveFullPower;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.DriveSlowPower;

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
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        intake.extend();

    }
    @Override
    public void loop() {
        super.loop();

        loopTime = systemClock.seconds();
        double matchTime = loopTime - initTime;

        if (manualShoot) {
            //whenever manual shooting or vertical intake running, drive slow
            drive.driveFieldCentric(gamepad1, DriveSlowPower, Auto.getAlliance());
            driveModeButton.resetToggle();

        } else {
//            if (IntakeController.numRings.get() >= 3 && matchTime < 85) {
//                shootButton.runOnceBlocking(true, this::autoShot, () -> shootButton.resetToggle());
//            }
            shootButton.runOnceBlocking(gameMap.Shoot(), this::autoShot, () -> shootButton.resetToggle());
            shootManButton.runOnce(gameMap.ShootManual(), this::manualShot);
            //TODO: FIX OPMODE STUCK LOOP TIMEOUT, USE ITERATIVE OP MODE
//            if  (VertIntakeController.isRunning && !IntakeController.isRunning) drive.driveFieldCentric(gamepad1, DriveSlowPower, Auto.getAlliance());
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
        }

        drive.update();

        incrementLeftButton.runOnceBlocking(
                gameMap.IncrementLeft(),
                () -> MechConstants.Red.incrementGoalShotAngle(0.4)
        );

        incrementRightButton.runOnceBlocking(
                gameMap.IncrementRight(),
                () -> MechConstants.Red.incrementGoalShotAngle(-0.4)
        );

        intakeButton.toggle(
                gameMap.Intake(),
                () -> intake.run(FORWARD),
                () -> intake.run(REVERSE),
                () -> intake.stopIntake());

        vertIntakeButton.toggle(
                gameMap.VertIntake(),
                () -> verticalIntake.run(FORWARD),
                () -> verticalIntake.run(REVERSE),
                () -> verticalIntake.stop());

        sweepFloorButton.toggle(
                gameMap.SweepFloor(),
                () -> verticalIntake.sweepOut(),
                () -> verticalIntake.sweepIn());

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
                () -> verticalIntake.stop(),
                () -> intakeButton.resetToggle(),
                () -> vertIntakeButton.resetToggle()
        );

        drive.putPacketData("intake sensor", intake.getSensorReading());
        drive.putPacketData("shooter RPM", shooter.getCurrentRPM());
        drive.putPacketData("target RPM", shooter.getTargetRPM());
//        drive.putPacketData("Num rings intaked", IntakeController.getNumRings());
        telemetry.addData("Rings Intaked", IntakeController.getNumRings());
        telemetry.addLine("<strong>Goal angle: </strong>" + MechConstants.Red.GoalShotAngle);
        telemetry.addData("Rings: Aspect Ratio", VerticalRingDetector.getAspectRatio());
        telemetry.addData("Rings: Width", VerticalRingDetector.getRingWidth());
        telemetry.addLine("<strong>Match time: </strong>" + Math.round(matchTime));
        telemetry.addLine("<strong>Using: </strong>" + hub.getFormattedCurrentDraw());
        telemetry.addLine("<strong>Loop Time: </strong>" + Math.round(systemClock.seconds() * 1000 - loopTime  * 1000) + " ms");
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
