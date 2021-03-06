package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;
import org.firstinspires.ftc.teamcode.opmodes.tele.params.GamepadMappings;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;
import org.firstinspires.ftc.teamcode.robot.camera.CameraController;
import org.firstinspires.ftc.teamcode.robot.camera.libs.OpenCVController;
import org.firstinspires.ftc.teamcode.robot.camera.libs.OpenCVController.PIPELINE;
import org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController;
import org.firstinspires.ftc.teamcode.robot.systems.HubController;
import org.firstinspires.ftc.teamcode.robot.systems.IntakeController;
import org.firstinspires.ftc.teamcode.robot.systems.ShooterController;
import org.firstinspires.ftc.teamcode.robot.systems.VertIntakeController;
import org.firstinspires.ftc.teamcode.robot.systems.WobbleController;
import org.firstinspires.ftc.teamcode.util.Button;


@Disabled
public class OpModeBase extends OpMode {
    protected GamepadMappings gameMap;
    protected Button intakeButton = new Button();
    protected Button vertIntakeButton = new Button();
    protected Button autoIntakeButton = new Button();
    protected Button stopIntakeButton = new Button();
    protected Button localizeButton = new Button();
    protected Button wobbleButton = new Button();
    protected Button wobbleDeliverButton = new Button();
    protected Button spinUpButton = new Button();
    protected Button shootButton = new Button();
    protected Button shootManButton = new Button();
    protected Button driveModeButton = new Button();

    protected Button wobbleAlignButton = new Button();

    protected DrivetrainController drive;
    protected IntakeController intake;
    protected ShooterController shooter;
    protected VertIntakeController vertIntake;
    protected WobbleController wobble;
    protected HubController hub;
    protected CameraController camera;

    protected ControllerManager controllers;
    protected MultipleTelemetry telemetry;
    protected NanoClock systemClock = NanoClock.system();

    protected enum OPMODE{
        Tele, Auto
    }

    protected OPMODE OPMODE_TYPE = OPMODE.Tele; //default

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

//        Properties properties = new Properties();
//        try {
//            properties.load(this.getClass().getClassLoader().getResourceAsStream("/version.properties"));
//            if (properties != null) {
//                String patch = properties.getProperty("VERSION_PATCH");
//                String build = properties.getProperty("VERSION_BUILD");
//                telemetry.addLine("Running Patch: " + patch + ", Build: " + build);
//                Log.i("Status, ","Running Patch: " + patch + ", Build: " + build);
//            }
//
//        } catch (IOException e) {
//            e.printStackTrace();
//        }

        telemetry.addLine("<strong>Initializing...</strong>");

        if (OPMODE_TYPE != null && OPMODE_TYPE == OPMODE.Auto) OpenCVController.DEFAULT_PIPELINE = PIPELINE.AUTO;
        else OpenCVController.DEFAULT_PIPELINE = PIPELINE.TELE;

        controllers = new ControllerManager(telemetry);
        controllers.make(hardwareMap, telemetry);

        drive = controllers.get(DrivetrainController.class, FieldConstants.Drive);
        hub = controllers.get(HubController.class, FieldConstants.Hub);
        shooter = controllers.get(ShooterController.class, FieldConstants.Shooter);
        intake = controllers.get(IntakeController.class, FieldConstants.Intake);
        vertIntake = controllers.get(VertIntakeController.class, FieldConstants.VertIntake);
        wobble = controllers.get(WobbleController.class, FieldConstants.Wobble);
        camera = controllers.get(CameraController.class, FieldConstants.Camera);

        controllers.init();

        gameMap = new GamepadMappings(gamepad1, gamepad2);

        telemetry.addLine("<h3>Initialized</h3>");
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        telemetry.clear();
        controllers.start();
    }


    @Override
    public void loop() {
        telemetry.addLine("<strong>Threads Running: </strong>" + java.lang.Thread.activeCount());
    }

    @Override
    public void stop() {
        telemetry.clear();

        telemetry.addLine("<strong>Stopping...</strong>");

        controllers.stop();

        telemetry.addLine("<h3>Stopped</h3>");
    }

}
