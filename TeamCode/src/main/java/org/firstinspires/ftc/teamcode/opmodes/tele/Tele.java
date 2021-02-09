package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.OpModeBase;
import org.firstinspires.ftc.teamcode.opmodes.auto.Auto;
import org.firstinspires.ftc.teamcode.opmodes.tele.params.GamepadMappings;
import org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants;
import org.firstinspires.ftc.teamcode.robot.camera.libs.OpenCVController;

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
    protected boolean manShoot = false;
    private double loopTime;
    protected int powerShotCt = 0;

    @Override
    public void init() {
        super.init();
        OpenCVController.DEFAULT_PIPELINE = OpenCVController.PIPELINE.TELE;
        drive.setPoseEstimate(MechConstants.StartingPose);
    }

    public void init_loop() {
        super.init_loop();
    }

    public void start() {
        super.start();
        intake.extend();
    }


    public void loop() {
        loopTime = systemClock.seconds() * 1000;

        super.loop();

        //lock out the gamepad during automatic shooting
        if (!autoShoot) {
            //automatically go to slow mode during manual shooting
            if (!manShoot) {
                driveModeButton.toggleLoop(
                        gameMap.DriveMode(),
                        () -> drive.driveFieldCentric(gamepad1, DriveFullPower, Auto.alliance),
                        () -> drive.driveFieldCentric(gamepad1, DriveSlowPower, Auto.alliance)
                );

            } else {
                if (!shooter.shootingState) manShoot = false;
                drive.driveFieldCentric(gamepad1, DriveSlowPower, Auto.alliance);
                driveModeButton.resetToggle();
            }
        }
        drive.update();

        if (!autoShoot) {
            shootButton.runOnce(gameMap.Shoot(), this::autoShoot);
        }
        shootManButton.runOnce(gameMap.ShootMan(), this::manShoot);

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
                () -> wobble.lift());

        wobbleGripButton.toggle(
                gameMap.WobbleGrip(),
                () -> wobble.grab(),
                () -> wobble.release());

        flywheelButton.toggle(
                gameMap.StartFlywheel(),
                () -> shooter.spinUp(RPMGoal),
                ()-> shooter.stop());

        wobbleAutoButton.runOnce(
                gameMap.WobbleAuto(),
                () -> drive.alignWithWobble(camera));

        if (gameMap.Shoot()){
            flywheelButton.resetToggle();
        }

        if (gameMap.StopAllIntakes()) {
            intake.stop();
            vertIntake.stop();
            intakeButton.resetToggle();
            vertIntakeButton.resetToggle();
        }

//        multiTelemetry.addLine(hub.getFormattedCurrentDraw());
//        multiTelemetry.addData("Loop Time",Math.round(clock.seconds() * 1000 - loopTime) + " ms");
    }

    protected abstract void autoShoot();
    protected abstract void powerShot();
    protected abstract void manShoot();

    public void stop() {
        super.stop();
    }

}
