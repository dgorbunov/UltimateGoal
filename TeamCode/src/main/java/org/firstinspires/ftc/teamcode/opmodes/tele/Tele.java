package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;
import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;
import org.firstinspires.ftc.teamcode.opmodes.tele.params.TeleConstants;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;
import org.firstinspires.ftc.teamcode.robot.camera.CameraController;
import org.firstinspires.ftc.teamcode.robot.drive.DriveLocalizationController;
import org.firstinspires.ftc.teamcode.robot.systems.HubController;
import org.firstinspires.ftc.teamcode.robot.systems.IntakeController;
import org.firstinspires.ftc.teamcode.robot.systems.ShooterController;
import org.firstinspires.ftc.teamcode.robot.systems.VertIntakeController;
import org.firstinspires.ftc.teamcode.robot.systems.WobbleController;

@TeleOp(name="Tele", group="Iterative Opmode")
@Config //for FTCDash
public abstract class Tele extends OpMode {

    public static volatile TeleConstants.DriverMode DriverMode = TeleConstants.DriverMode.OneDriver;

    private DriveLocalizationController drive;
    private IntakeController intake;
    private ShooterController shooter;
    private VertIntakeController vertIntake;
    private WobbleController wobble;
    private HubController hub;
    private CameraController camera;

    private TeleConstants gp;

    private ControllerManager controllers;

    public void init() {
        telemetry.addLine("Initializing...");

        DriveLocalizationController.TESTING = true;

        controllers = new ControllerManager(telemetry);
        controllers.make(hardwareMap, telemetry);

        drive = controllers.get(DriveLocalizationController.class, FieldConstants.Drive);
        hub = controllers.get(HubController.class, FieldConstants.Hub);
        shooter = controllers.get(ShooterController.class, FieldConstants.Shooter);
        intake = controllers.get(IntakeController.class, FieldConstants.Intake);
        vertIntake = controllers.get(VertIntakeController.class, FieldConstants.VertIntake);
        wobble = controllers.get(WobbleController.class, FieldConstants.Wobble);
        camera = controllers.get(CameraController.class, FieldConstants.Camera);

        controllers.init();

        telemetry.clear();
        telemetry.addLine("Initialized");
    }

    public void init_loop() {

    }

    public void start() {
        controllers.start();
        TeleConstants.setGamepads(gamepad1, gamepad2);
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
        if (TeleConstants.StartFlywheel) shooter.spinUp(4800);
        if (TeleConstants.Shoot) shooter.shoot( 3); //TODO: autoalign

        if (TeleConstants.IntakeForward) intake.run(0.8);
        if (TeleConstants.IntakeReverse) intake.run(-0.8); //TODO: multithread all of this
        if (TeleConstants.VertIntakeForward) vertIntake.run(1.0);
        if (TeleConstants.VertIntakeReverse) vertIntake.run(-1.0);
        if (TeleConstants.StopAllIntakes) {
            intake.stop();
            vertIntake.stop();
        }
        if (TeleConstants.PickupWobble) wobble.pickup();
        if (TeleConstants.DropWobble) wobble.drop();

        telemetry.addLine(hub.getFormattedCurrentDraw());
    }

    protected abstract void autoShoot();

    public void stop() { //code to run when program is stopped
        telemetry.addLine("Stopping...");

        controllers.stop();

        telemetry.clear();
        telemetry.addLine("Stopped");
    }

}
