package org.firstinspires.ftc.teamcode.opmodes.tele.base;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;
import org.firstinspires.ftc.teamcode.deprecated.DrivetrainController;
import org.firstinspires.ftc.teamcode.robot.drive.DriveLocalizationController;
import org.firstinspires.ftc.teamcode.robot.systems.HubController;
import org.firstinspires.ftc.teamcode.robot.systems.IntakeController;
import org.firstinspires.ftc.teamcode.robot.systems.ShooterController;
import org.firstinspires.ftc.teamcode.robot.systems.VIntakeController;
import org.firstinspires.ftc.teamcode.robot.systems.WobbleController;

@TeleOp(name="Tele", group="Iterative Opmode")
public abstract class Tele extends OpMode {

    public Tele(){

    }

    private DriveLocalizationController drive;
    private IntakeController intake;
    private ShooterController shooter;
    private VIntakeController vertIntake;
    private WobbleController wobble;
    private HubController hub;

    private TeleConstants gp;

    protected String alliance;

    private ControllerManager controllers;

    public void init() {
        telemetry.addLine("Initializing...");

        controllers = new ControllerManager(telemetry);

        drive = new DriveLocalizationController(hardwareMap, telemetry);
        hub = new HubController(hardwareMap, telemetry);
        shooter = new ShooterController(hardwareMap, telemetry);
        intake = new IntakeController(hardwareMap, telemetry);
        vertIntake = new VIntakeController(hardwareMap, telemetry);
        wobble = new WobbleController(hardwareMap, telemetry);


        controllers.add(FieldConstants.Drive, drive);
        controllers.add(FieldConstants.Hub, hub);
        controllers.add(FieldConstants.Shooter, shooter);
        controllers.add(FieldConstants.Intake, intake);
        controllers.add(FieldConstants.VertIntake, vertIntake);
        controllers.add(FieldConstants.Wobble, wobble);

        controllers.init();

        telemetry.clear();
        telemetry.addLine("Initialized");
    }

    public void init_loop() {

    }

    public void start() {
        controllers.start();
        gp = new TeleConstants(gamepad1, gamepad2); //will this work if
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

        telemetry.addLine(hub.getFormattedCurrentDraw());
    }

    protected abstract void autoShoot();

    public void stop() { //code to run when program is stopped
        telemetry.addLine("Stopping...");

        controllers.stop();

        telemetry.clear();
        telemetry.addLine("Stopped");
    }

//    public static Telemetry GetTelemetry() {
//        return sTelemetry;
//    }
}
