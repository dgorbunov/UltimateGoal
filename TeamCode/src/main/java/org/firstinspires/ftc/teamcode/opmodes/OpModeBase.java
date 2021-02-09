package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;
import org.firstinspires.ftc.teamcode.opmodes.tele.params.GamepadMappings;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;
import org.firstinspires.ftc.teamcode.robot.camera.CameraController;
import org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController;
import org.firstinspires.ftc.teamcode.robot.systems.HubController;
import org.firstinspires.ftc.teamcode.robot.systems.IntakeController;
import org.firstinspires.ftc.teamcode.robot.systems.ShooterController;
import org.firstinspires.ftc.teamcode.robot.systems.VertIntakeController;
import org.firstinspires.ftc.teamcode.robot.systems.WobbleController;
import org.firstinspires.ftc.teamcode.util.Button;

@TeleOp(name="OpModeBase", group="Iterative Opmode")
@Disabled
public class OpModeBase extends OpMode {

    protected GamepadMappings gameMap;
    protected Button intakeButton = new Button();
    protected Button vertIntakeButton = new Button();
    protected Button wobbleArmButton = new Button();
    protected Button wobbleGripButton = new Button();
    protected Button flywheelButton = new Button();
    protected Button shootButton = new Button();
    protected Button driveModeButton = new Button();
    protected Button shootManButton = new Button();
    protected Button wobbleAutoButton = new Button();

    protected DrivetrainController drive;
    protected IntakeController intake;
    protected ShooterController shooter;
    protected VertIntakeController vertIntake;
    protected WobbleController wobble;
    protected HubController hub;
    protected CameraController camera;

    protected ControllerManager controllers;
    protected MultipleTelemetry multiTelemetry;
    protected NanoClock systemClock = NanoClock.system();

    @Override
    public void init() {
        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        multiTelemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        multiTelemetry.addLine("Initializing...");

        controllers = new ControllerManager(multiTelemetry);
        controllers.make(hardwareMap, multiTelemetry);

        drive = controllers.get(DrivetrainController.class, FieldConstants.Drive);
        hub = controllers.get(HubController.class, FieldConstants.Hub);
        shooter = controllers.get(ShooterController.class, FieldConstants.Shooter);
        intake = controllers.get(IntakeController.class, FieldConstants.Intake);
        vertIntake = controllers.get(VertIntakeController.class, FieldConstants.VertIntake);
        wobble = controllers.get(WobbleController.class, FieldConstants.Wobble);

        gameMap = new GamepadMappings(gamepad1, gamepad2);

        multiTelemetry.addLine("Initialized");
        multiTelemetry.setDisplayFormat(Telemetry.DisplayFormat.CLASSIC);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        controllers.start();
        intake.extend();
        multiTelemetry.clear();
    }


    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        multiTelemetry.clear();
        multiTelemetry.addLine("Stopping...");

        controllers.stop();

        multiTelemetry.clear();
        multiTelemetry.addLine("Stopped");
    }
}
