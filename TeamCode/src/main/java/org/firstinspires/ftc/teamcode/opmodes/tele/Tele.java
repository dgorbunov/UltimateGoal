package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.OpModeBase;
import org.firstinspires.ftc.teamcode.opmodes.auto.Auto;
import org.firstinspires.ftc.teamcode.opmodes.tele.params.GamepadMappings;
import org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants;
import org.firstinspires.ftc.teamcode.robot.camera.algorithms.VerticalRingDetector;
import org.firstinspires.ftc.teamcode.robot.systems.VertIntakeController;
import org.firstinspires.ftc.teamcode.util.Button;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.DriveFullPower;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.DriveSlowPower;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.RPMGoal;
import static org.firstinspires.ftc.teamcode.robot.camera.CameraController.Objects.VERTICAL_RING;
import static org.firstinspires.ftc.teamcode.robot.camera.CameraController.Objects.WOBBLE;

@TeleOp(name="Tele", group="Iterative Opmode")
@Disabled
public abstract class Tele extends OpModeBase {

    public static volatile GamepadMappings.DriverMode DriverMode = GamepadMappings.DriverMode.TwoDrivers;

    protected boolean autoShoot = false;
    protected boolean manualShoot = false;
    private double loopTime;

    public Tele() {
        super();
    }

    @Override
    public void init() {
        OPMODE_TYPE = OPMODE.Tele;
        super.init();

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
        loopTime = systemClock.seconds() * 1000;

        if (!manualShoot && !autoShoot && !VertIntakeController.isRunning) {
            shootButton.runOnce(gameMap.Shoot(), this::autoShot, () -> shootButton.resetToggle());
            shootManButton.runOnce(gameMap.ShootManual(), this::manualShot);

            driveModeButton.toggleLoop(
                    gameMap.DriveMode(),
                    () -> drive.driveFieldCentric(gamepad1, DriveFullPower, Auto.getAlliance()),
                    () -> drive.driveFieldCentric(gamepad1, DriveSlowPower, Auto.getAlliance())
            );
        } else {
            drive.driveFieldCentric(gamepad1, DriveSlowPower, Auto.getAlliance());
            driveModeButton.resetToggle();
        }

        drive.update();

        intakeButton.toggle(
                gameMap.Intake(),
                () -> intake.run(FORWARD),
                () -> intake.run(REVERSE),
                () -> intake.stopIntake());

        vertIntakeButton.toggle(
                gameMap.VertIntake(),
                () -> vertIntake.run(FORWARD),
                () -> vertIntake.run(REVERSE),
                () -> vertIntake.stop());

        wobbleButton.toggle(
                gameMap.Wobble(),
                () -> wobble.dropTele(),
                () -> wobble.pickupTele());

        wobbleDeliverButton.toggle(
                gameMap.WobbleDeliver(),
                () -> wobble.release());

        spinUpButton.toggle(
                gameMap.SpinUp(),
                () -> shooter.spinUp(RPMGoal),
                ()-> shooter.stop());

        wobbleAlignButton.toggle(
                gameMap.WobbleAlign(),
                () -> drive.alignWithObject(camera, WOBBLE),
                () -> drive.stop());

        autoIntakeButton.toggle(
                gameMap.AutoVertIntake(),
                () -> new Button().runAllBlocking(
                        () -> drive.turnAbsolute(0),
                        () -> vertIntake.run(FORWARD),
                        () -> drive.alignWithObject(camera, VERTICAL_RING))
                );

        localizeButton.runOnce(
                gameMap.Localize(),
                this::localizeWithCorner
        );

        stopIntakeButton.runOnce(
                gameMap.StopAllIntakes(),
                () -> intake.stopIntake(),
                () -> vertIntake.stop(),
                () -> intakeButton.resetToggle(),
                () -> vertIntakeButton.resetToggle()
        );

        telemetry.addData("Rings: Aspect Ratio", VerticalRingDetector.getAspectRatio());
        telemetry.addData("Rings: Width", VerticalRingDetector.getRingWidth());
        telemetry.addLine("<strong>Using: </strong>" + hub.getCurrentDraw());
        telemetry.addLine("<strong>Loop Time: </strong>" + Math.round(systemClock.seconds() * 1000 - loopTime) + " ms");
    }

    protected abstract void autoShot();
    protected abstract void manualPowerShot();
    protected abstract void manualShot();
    protected abstract void localizeWithCorner();

    @Override
    public void stop() {
        super.stop();
    }

}
