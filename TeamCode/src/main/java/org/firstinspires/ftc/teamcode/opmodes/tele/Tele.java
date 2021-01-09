package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;
import org.firstinspires.ftc.teamcode.opmodes.tele.params.GamepadMappings;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;
import org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController;
import org.firstinspires.ftc.teamcode.robot.systems.HubController;
import org.firstinspires.ftc.teamcode.robot.systems.IntakeController;
import org.firstinspires.ftc.teamcode.robot.systems.ShooterController;
import org.firstinspires.ftc.teamcode.robot.systems.VertIntakeController;
import org.firstinspires.ftc.teamcode.robot.systems.WobbleController;
import org.firstinspires.ftc.teamcode.util.Button;

import static org.firstinspires.ftc.teamcode.opmodes.tele.params.TeleConstants.IntakeForwardPower;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.TeleConstants.IntakeReversePower;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.TeleConstants.VertIntakeForwardPower;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.TeleConstants.VertIntakeReversePower;

@TeleOp(name="Tele", group="Iterative Opmode")
@Disabled
@Config //for FTCDash
public abstract class Tele extends OpMode {

    public static volatile GamepadMappings.DriverMode DriverMode = GamepadMappings.DriverMode.OneDriver;

    GamepadMappings gameMap = new GamepadMappings(gamepad1, gamepad2);
    Button intakeButton;
    Button vertIntakeButton;
    Button wobbleButton;
    Button flywheelButton;
    Button shootButton;

    protected DrivetrainController drive;
    protected IntakeController intake;
    protected ShooterController shooter;
    protected VertIntakeController vertIntake;
    protected WobbleController wobble;
    protected HubController hub;
    //private CameraController camera;

    protected ControllerManager controllers;

    public void init() {
        telemetry.addLine("Initializing...");

        DrivetrainController.TESTING = true;

        controllers = new ControllerManager(telemetry);
        controllers.make(hardwareMap, telemetry);

        drive = controllers.get(DrivetrainController.class, FieldConstants.Drive);
        hub = controllers.get(HubController.class, FieldConstants.Hub);
        shooter = controllers.get(ShooterController.class, FieldConstants.Shooter);
        intake = controllers.get(IntakeController.class, FieldConstants.Intake);
        vertIntake = controllers.get(VertIntakeController.class, FieldConstants.VertIntake);
        wobble = controllers.get(WobbleController.class, FieldConstants.Wobble);

        controllers.init();

//        gameMap.setGamepads(gamepad1, gamepad2);


        wobbleButton = new Button();
        flywheelButton = new Button();
        vertIntakeButton = new Button();
        intakeButton = new Button();
        shootButton = new Button();

        telemetry.clear();
        telemetry.addLine("Initialized");
    }

    public void init_loop() {

    }

    public void start() {
        controllers.start();
    }

    public void loop() {
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );

        drive.update();

        shootButton.runOnce(gameMap.ShootButton(), () -> shooter.shoot(3));

        intakeButton.toggle(
                gameMap.IntakeButton(),
                () -> intake.run(IntakeForwardPower),
                () -> intake.run(IntakeReversePower),
                () -> intake.stop());

        vertIntakeButton.toggle(
                gameMap.VertIntakeButton(),
                () -> vertIntake.run(VertIntakeForwardPower),
                () -> vertIntake.run(VertIntakeReversePower),
                () -> vertIntake.stop());

        wobbleButton.toggle(
                gameMap.VertIntakeButton(),
                () -> wobble.pickup(),
                () -> wobble.drop());

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
