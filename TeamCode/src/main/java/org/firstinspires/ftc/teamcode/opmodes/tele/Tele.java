package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;
import org.firstinspires.ftc.teamcode.opmodes.tele.params.GamepadMappings;
import org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;
import org.firstinspires.ftc.teamcode.robot.camera.CameraController;
import org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController;
import org.firstinspires.ftc.teamcode.robot.systems.HubController;
import org.firstinspires.ftc.teamcode.robot.systems.IntakeController;
import org.firstinspires.ftc.teamcode.robot.systems.ShooterController;
import org.firstinspires.ftc.teamcode.robot.systems.VertIntakeController;
import org.firstinspires.ftc.teamcode.robot.systems.WobbleController;
import org.firstinspires.ftc.teamcode.util.Button;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.DriveFullPower;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.DriveSlowPower;
import static org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants.RPMGoal;

@TeleOp(name="Tele", group="Iterative Opmode")
@Disabled
@Config
public abstract class Tele extends OpMode {

    public static volatile GamepadMappings.DriverMode DriverMode = GamepadMappings.DriverMode.OneDriver;

    GamepadMappings gameMap;
    Button intakeButton = new Button();
    //TODO: test, make all gameMap buttons Booleans
    Button vertIntakeButton = new Button();
    Button wobbleArmButton = new Button();
    Button wobbleGripButton = new Button();
    Button flywheelButton = new Button();
    Button shootButton = new Button();
    Button driveModeButton = new Button();
    Button shootManButton = new Button();

    protected DrivetrainController drive;
    protected IntakeController intake;
    protected ShooterController shooter;
    protected VertIntakeController vertIntake;
    protected WobbleController wobble;
    protected HubController hub;
    protected CameraController camera;

    protected ControllerManager controllers;

    protected boolean autoShoot = false;
    protected boolean manShoot = false;

    MultipleTelemetry data;

    public void init() {
        //TODO: how do you make this just telemetry?
        data = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.addLine("Initializing...");

        DrivetrainController.TESTING = false;

        controllers = new ControllerManager(telemetry);
        controllers.make(hardwareMap, telemetry);

        drive = controllers.get(DrivetrainController.class, FieldConstants.Drive);
        hub = controllers.get(HubController.class, FieldConstants.Hub);
        shooter = controllers.get(ShooterController.class, FieldConstants.Shooter);
        intake = controllers.get(IntakeController.class, FieldConstants.Intake);
        vertIntake = controllers.get(VertIntakeController.class, FieldConstants.VertIntake);
        wobble = controllers.get(WobbleController.class, FieldConstants.Wobble);

        controllers.init();

        drive.setPoseEstimate(MechConstants.StartingPose);

//        gameMap.setGamepads(gamepad1, gamepad2);
        gameMap = new GamepadMappings(gamepad1, gamepad2);

        telemetry.clear();
        telemetry.addLine("Initialized");
    }

    public void init_loop() {

    }

    public void start() {
        controllers.start();
        intake.extend();
    }


    public void loop() {
        //lock out the gamepad during automatic shooting
        if (!autoShoot) {
            //automatically go to slow mode during manual shooting
            if (!manShoot) {
                driveModeButton.toggleLoop(
                        gameMap.DriveMode(),
                        () -> drive.driveFieldCentric(gamepad1, DriveFullPower),
                        () -> drive.driveFieldCentric(gamepad1, DriveSlowPower)
                );

            } else {
                if (!shooter.shootingState) manShoot = false;
                drive.driveFieldCentric(gamepad1, DriveSlowPower);
                driveModeButton.resetToggle();
            }
        }

        drive.update();

        shootButton.runOnce(gameMap.Shoot(), this::autoShoot);
        shootManButton.runOnce(gameMap.ShootMan(), this::manShoot);

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
                () -> shooter.spinUp(RPMGoal),
                ()-> shooter.stop());

        if (gameMap.Shoot()){
            flywheelButton.resetToggle();
        }


        if (gameMap.StopAllIntakes()) {
            intake.stop();
            vertIntake.stop();
            intakeButton.resetToggle();
            vertIntakeButton.resetToggle();
        }

        telemetry.addLine(hub.getFormattedCurrentDraw());
    }

    protected abstract void autoShoot();
    protected abstract void powerShot();
    protected abstract void manShoot();

    public void stop() {
        telemetry.addLine("Stopping...");

        controllers.stop();

        telemetry.clear();
        telemetry.addLine("Stopped");
    }

}
