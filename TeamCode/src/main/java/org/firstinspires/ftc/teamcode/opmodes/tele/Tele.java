package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;
import org.firstinspires.ftc.teamcode.opmodes.tele.params.GamepadMappings;
import org.firstinspires.ftc.teamcode.opmodes.tele.params.TeleConstants;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;
import org.firstinspires.ftc.teamcode.robot.camera.CameraController;
import org.firstinspires.ftc.teamcode.robot.systems.HubController;
import org.firstinspires.ftc.teamcode.robot.systems.IntakeController;
import org.firstinspires.ftc.teamcode.robot.systems.ShooterController;
import org.firstinspires.ftc.teamcode.robot.systems.VertIntakeController;
import org.firstinspires.ftc.teamcode.robot.systems.WobbleController;
import org.firstinspires.ftc.teamcode.util.Button;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

@TeleOp(name="Tele", group="Iterative Opmode")
@Disabled
@Config //for FTCDash
public abstract class Tele extends OpMode {

    public static volatile GamepadMappings.DriverMode DriverMode = GamepadMappings.DriverMode.OneDriver;

    GamepadMappings gameMap = new GamepadMappings();
    Button intakeButton = new Button();
    Button vertIntakeButton = new Button();
    Button wobbleArmButton = new Button();
    Button wobbleGripButton = new Button();
    Button flywheelButton = new Button();
    Button shootButton = new Button();
    Button driveModeButton = new Button();
    Button extendIntakeButton = new Button();

    protected org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController drive;
    protected IntakeController intake;
    protected ShooterController shooter;
    protected VertIntakeController vertIntake;
    protected WobbleController wobble;
    protected HubController hub;
    protected CameraController camera;

    protected ControllerManager controllers;

    public void init() {
        telemetry.addLine("Initializing...");

        org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController.TESTING = false;

        controllers = new ControllerManager(telemetry);
        controllers.make(hardwareMap, telemetry);

        drive = controllers.get(org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController.class, FieldConstants.Drive);
        hub = controllers.get(HubController.class, FieldConstants.Hub);
        shooter = controllers.get(ShooterController.class, FieldConstants.Shooter);
        intake = controllers.get(IntakeController.class, FieldConstants.Intake);
        vertIntake = controllers.get(VertIntakeController.class, FieldConstants.VertIntake);
        wobble = controllers.get(WobbleController.class, FieldConstants.Wobble);

        controllers.init();

        gameMap.setGamepads(gamepad1, gamepad2);

        telemetry.clear();
        telemetry.addLine("Initialized");
    }

    public void init_loop() {

    }

    public void start() {
        controllers.start();
    }


    public void loop() {
        driveModeButton.toggleLoop(
                gameMap.DriveMode(),
                () -> drive.setWeightedDrivePower(
                new Pose2d(
                        TeleConstants.DriveFullPower * -gamepad1.left_stick_y,
                        TeleConstants.DriveFullPower * -gamepad1.left_stick_x,
                        TeleConstants.DriveFullPower * -gamepad1.right_stick_x
                )),
                () -> drive.setWeightedDrivePower(
                new Pose2d(
                        TeleConstants.DriveSlowPower * -gamepad1.left_stick_y,
                        TeleConstants.DriveSlowPower * -gamepad1.left_stick_x,
                        TeleConstants.DriveSlowPower * -gamepad1.right_stick_x
                )
        ));

        drive.update();

        shootButton.runOnce(gameMap.Shoot(), () -> shooter.shoot(3));
        extendIntakeButton.runOnce(gameMap.ExtendIntake(), () -> intake.extend());

        intakeButton.toggle(
                gameMap.Intake(),
                () -> intake.run(FORWARD),
                () -> intake.run(REVERSE),
                () -> intake.stop());

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
                () -> shooter.spinUp(4800),
                ()-> shooter.stop());



        if (gameMap.StopAllIntakes()) {
            intake.stop();
            vertIntake.stop();
            intakeButton.resetToggle();
            vertIntakeButton.resetToggle();
        }

        telemetry.addLine(hub.getFormattedCurrentDraw());
    }

    protected abstract void autoShoot();

    public void stop() {
        telemetry.addLine("Stopping...");

        controllers.stop();

        telemetry.clear();
        telemetry.addLine("Stopped");
    }

}
