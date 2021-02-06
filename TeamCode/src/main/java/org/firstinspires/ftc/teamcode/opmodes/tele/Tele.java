package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.auto.Auto;
import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;
import org.firstinspires.ftc.teamcode.opmodes.tele.params.GamepadMappings;
import org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;
import org.firstinspires.ftc.teamcode.robot.camera.CameraController;
import org.firstinspires.ftc.teamcode.robot.camera.libs.OpenCVController;
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
public abstract class Tele extends OpMode {

    public static volatile GamepadMappings.DriverMode DriverMode = GamepadMappings.DriverMode.OneDriver;

    GamepadMappings gameMap;
    Button intakeButton = new Button();
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
    protected MultipleTelemetry multiTelemetry;
    private NanoClock clock = NanoClock.system();

    protected boolean autoShoot = false;
    protected boolean manShoot = false;
    private double loopTime;
    protected int powerShotCt = 0;

    public void init() {
        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        multiTelemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        multiTelemetry.addLine("Initializing...");

        DrivetrainController.TESTING = false;
        OpenCVController.DEFAULT_PIPELINE = OpenCVController.PIPELINE.TELE;

        controllers = new ControllerManager(multiTelemetry);
        controllers.make(hardwareMap, multiTelemetry);

        drive = controllers.get(DrivetrainController.class, FieldConstants.Drive);
        hub = controllers.get(HubController.class, FieldConstants.Hub);
        shooter = controllers.get(ShooterController.class, FieldConstants.Shooter);
        intake = controllers.get(IntakeController.class, FieldConstants.Intake);
        vertIntake = controllers.get(VertIntakeController.class, FieldConstants.VertIntake);
        wobble = controllers.get(WobbleController.class, FieldConstants.Wobble);

        controllers.init();

        drive.setPoseEstimate(MechConstants.StartingPose);

        gameMap = new GamepadMappings(gamepad1, gamepad2);

        //TODO: test telemetry.DisplayFormat with HTML tags
        multiTelemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        multiTelemetry.addLine("<strong> Initialized </strong>");
        multiTelemetry.setDisplayFormat(Telemetry.DisplayFormat.CLASSIC);
    }

    public void init_loop() {

    }

    public void start() {
        controllers.start();
        intake.extend();
        multiTelemetry.clear();
    }


    public void loop() {
        loopTime = clock.seconds() * 1000;

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

        if (gameMap.Shoot()){
            flywheelButton.resetToggle();
        }


        if (gameMap.StopAllIntakes()) {
            intake.stop();
            vertIntake.stop();
            intakeButton.resetToggle();
            vertIntakeButton.resetToggle();
        }

        multiTelemetry.addLine(hub.getFormattedCurrentDraw());
        multiTelemetry.addData("Loop Time",Math.round(clock.seconds() * 1000 - loopTime) + " ms");
    }

    protected abstract void autoShoot();
    protected abstract void powerShot();
    protected abstract void manShoot();

    public void stop() {
        multiTelemetry.clear();
        multiTelemetry.addLine("Stopping...");

        controllers.stop();

        multiTelemetry.clear();
        multiTelemetry.addLine("Stopped");
    }

}
