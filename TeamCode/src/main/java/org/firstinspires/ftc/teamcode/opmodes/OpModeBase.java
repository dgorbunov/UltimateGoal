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
import org.firstinspires.ftc.teamcode.robot.systems.RearIntakeController;
import org.firstinspires.ftc.teamcode.robot.systems.ShooterController;
import org.firstinspires.ftc.teamcode.robot.systems.WobbleController;
import org.firstinspires.ftc.teamcode.util.Button;


@Disabled
public class OpModeBase extends OpMode {
    protected GamepadMappings gamepad;
    protected Button intakeButton = new Button();
    protected Button resetIntakeCounterButton = new Button();
    protected Button autoIntakeButton = new Button();
    protected Button stopIntakeButton = new Button();
    protected Button localizeButton = new Button();
    protected Button wobbleButton = new Button();
    protected Button wobbleDeliverButton = new Button();
    protected Button powerShotButton = new Button();
    protected Button shootButton = new Button();
    protected Button shootManButton = new Button();
    protected Button driveModeButton = new Button();
    protected Button incrementLeftButton = new Button();
    protected Button incrementRightButton = new Button();

    protected DrivetrainController drive;
    protected IntakeController intake;
    protected ShooterController shooter;
    protected RearIntakeController rearIntake;
    protected WobbleController wobble;
    protected HubController hub;
    protected CameraController camera;

    protected ControllerManager controllers;
    protected MultipleTelemetry telemetryd;
    protected NanoClock systemClock = NanoClock.system();

    public enum OPMODE{
        Tele, Auto
    }

    protected static OPMODE OPMODE_TYPE = OPMODE.Tele; //default

    @Override
    public void init() {
        telemetryd = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryd.setDisplayFormat(Telemetry.DisplayFormat.HTML);

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

        telemetryd.addLine("<strong>Initializing...</strong>");

        if (OPMODE_TYPE != null && OPMODE_TYPE == OPMODE.Auto) OpenCVController.DEFAULT_PIPELINE = PIPELINE.AUTO;
        else OpenCVController.DEFAULT_PIPELINE = PIPELINE.TELE;

        controllers = new ControllerManager(telemetry, telemetryd);
        controllers.make(hardwareMap, telemetry, telemetryd);

        drive = controllers.get(DrivetrainController.class, FieldConstants.Drive);
        hub = controllers.get(HubController.class, FieldConstants.Hub);
        shooter = controllers.get(ShooterController.class, FieldConstants.Shooter);
        intake = controllers.get(IntakeController.class, FieldConstants.Intake);
        rearIntake = controllers.get(RearIntakeController.class, FieldConstants.VertIntake);
        wobble = controllers.get(WobbleController.class, FieldConstants.Wobble);
        camera = controllers.get(CameraController.class, FieldConstants.Camera);

        controllers.init();

        gamepad = new GamepadMappings(gamepad1, gamepad2);

        telemetryd.addLine("<h3>Initialized</h3>");
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        telemetryd.clear();
        controllers.start();
    }


    @Override
    public void loop() {
        telemetryd.addLine("<strong>Threads Running: </strong>" + java.lang.Thread.activeCount());
    }

    @Override
    public void stop() {
        telemetryd.clear();
        telemetryd.addLine("<strong>Stopping...</strong>");

        controllers.stop();

        telemetryd.addLine("<h3>Stopped</h3>");
    }

    public static OPMODE getOpModeType() {
        return OPMODE_TYPE;
    }
}
