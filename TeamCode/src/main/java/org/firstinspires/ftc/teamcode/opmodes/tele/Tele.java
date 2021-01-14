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
    Button intakeButton;
    Button vertIntakeButton;
    Button wobbleButton;
    Button flywheelButton;
    Button shootButton;
    Button driveModeButton;

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


        wobbleButton = new Button();
        flywheelButton = new Button();
        vertIntakeButton = new Button();
        intakeButton = new Button();
        shootButton = new Button();
        driveModeButton = new Button();

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
                gameMap.DriveModeButton(),
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

        shootButton.runOnce(gameMap.ShootButton(), () -> shooter.shoot(3));

        intakeButton.toggle(
                gameMap.IntakeButton(),
                () -> intake.run(FORWARD),
                () -> intake.run(REVERSE),
                () -> intake.stop());

        vertIntakeButton.toggle(
                gameMap.VertIntakeButton(),
                () -> vertIntake.run(FORWARD),
                () -> vertIntake.run(REVERSE),
                () -> vertIntake.stop());

        wobbleButton.toggle(
                gameMap.VertIntakeButton(),
                () -> wobble.pickupAuto(),
                () -> wobble.dropAuto());

        flywheelButton.toggle(
                gameMap.FlywheelButton(),
                () -> shooter.spinUp(4800),
                ()-> shooter.stop());

        if (gameMap.StopIntakesButton()) {
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