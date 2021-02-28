package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.OpModeBase;
import org.firstinspires.ftc.teamcode.opmodes.auto.Auto;
import org.firstinspires.ftc.teamcode.opmodes.tele.params.GamepadMappings;
import org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants;
import org.firstinspires.ftc.teamcode.robot.camera.algorithms.WobbleDetector;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.DriveFullPower;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.DriveSlowPower;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.RPMGoal;

@TeleOp(name="Tele", group="Iterative Opmode")
@Disabled
public abstract class Tele extends OpModeBase {

    public static volatile GamepadMappings.DriverMode DriverMode = GamepadMappings.DriverMode.OneDriver;

    protected boolean autoShoot = false;
    protected boolean manualShoot = false;
    private double loopTime;
    protected int powerShotCt = 0;

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


        if (autoShoot) return;

        //automatically go to slow mode during shooting
        if (!manualShoot) {
            driveModeButton.toggleLoop(
                    gameMap.DriveMode(),
                    () -> drive.driveFieldCentric(gamepad1, DriveFullPower, Auto.getAlliance()),
                    () -> drive.driveFieldCentric(gamepad1, DriveSlowPower, Auto.getAlliance())
            );
        }
        else {
            drive.driveFieldCentric(gamepad1, DriveSlowPower, Auto.getAlliance());
            driveModeButton.resetToggle();
        }

        drive.update();

        if (!manualShoot) {
            shootButton.runOnce(gameMap.Shoot(), this::autoShoot);
            shootManButton.runOnce(gameMap.ShootManual(), this::manualShoot);
        }

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

        wobbleArmButton.toggle(
                gameMap.WobbleArm(),
                () -> wobble.drop(),
                () -> wobble.pickup());

        wobbleGripButton.toggle(
                gameMap.WobbleGrip(),
                () -> wobble.grab(),
                () -> wobble.release());

        flywheelButton.toggle(
                gameMap.StartFlywheel(),
                () -> shooter.spinUp(RPMGoal),
                ()-> shooter.stop());

        wobbleAutoButton.toggle(
                gameMap.WobbleAuto(),
                () -> drive.alignWithWobble(camera),
                () -> drive.stop());

        if (gameMap.Shoot()){
            flywheelButton.resetToggle();
        }

        if (gameMap.Localize()) {
            localize();
        }

        if (gameMap.StopAllIntakes()) {
            intake.stopIntake();
            vertIntake.stop();
            intakeButton.resetToggle();
            vertIntakeButton.resetToggle();
        }

        multiTelemetry.addData("Vision: Aspect Ratio", WobbleDetector.getAspectRatio());
        multiTelemetry.addLine(hub.getFormattedCurrentDraw());
        multiTelemetry.addData("Loop Time",Math.round(systemClock.seconds() * 1000 - loopTime) + " ms");
    }

    protected abstract void autoShoot();
    protected abstract void powerShot();
    protected abstract void manualShoot();
    protected abstract void localize();

    @Override
    public void stop() {
        super.stop();
    }

}
