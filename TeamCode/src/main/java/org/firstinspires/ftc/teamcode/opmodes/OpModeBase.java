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
    protected Button sweeperButton = new Button();
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

    protected enum OPMODE{
        Tele, Auto
    }

    protected OPMODE OPMODE_TYPE = OPMODE.Tele; //default

    @Override
    public void init() {
        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        multiTelemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);

//        Properties properties = new Properties();
//        try {
//            properties.load(this.getClass().getClassLoader().getResourceAsStream("/version.properties"));
//            if (properties != null) {
//                String patch = properties.getProperty("VERSION_PATCH");
//                String build = properties.getProperty("VERSION_BUILD");
//                multiTelemetry.addLine("Running Patch: " + patch + ", Build: " + build);
//                Log.i("Status, ","Running Patch: " + patch + ", Build: " + build);
//            }
//
//        } catch (IOException e) {
//            e.printStackTrace();
//        }

        multiTelemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        multiTelemetry.addLine("<strong style = \"color:DodgerBlue;\">Initializing...</strong>"); //should this be normal telemetry?
        multiTelemetry.setDisplayFormat(Telemetry.DisplayFormat.CLASSIC);
        //https://stackoverflow.com/questions/9754076/which-html-tags-are-supported-by-android-textview

        if (OPMODE_TYPE != null && OPMODE_TYPE == OPMODE.Auto) OpenCVController.DEFAULT_PIPELINE = PIPELINE.AUTO;
        else OpenCVController.DEFAULT_PIPELINE = PIPELINE.TELE;

        controllers = new ControllerManager(multiTelemetry);
        controllers.make(hardwareMap, multiTelemetry);

        drive = controllers.get(DrivetrainController.class, FieldConstants.Drive);
        hub = controllers.get(HubController.class, FieldConstants.Hub);
        shooter = controllers.get(ShooterController.class, FieldConstants.Shooter);
        intake = controllers.get(IntakeController.class, FieldConstants.Intake);
        vertIntake = controllers.get(VertIntakeController.class, FieldConstants.VertIntake);
        wobble = controllers.get(WobbleController.class, FieldConstants.Wobble);
        camera = controllers.get(CameraController.class, FieldConstants.Camera);

        controllers.init();

        gameMap = new GamepadMappings(gamepad1, gamepad2);

        multiTelemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        multiTelemetry.addLine("<strong style = \"color:DodgerBlue;\">Initialized</strong>");
        multiTelemetry.setDisplayFormat(Telemetry.DisplayFormat.CLASSIC);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        multiTelemetry.clear();
        controllers.start();
    }


    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        multiTelemetry.clear();

        multiTelemetry.addLine("<strong style = \"color:DodgerBlue;\">Stopping...</strong>");

        controllers.stop();

        multiTelemetry.addLine("<strong style = \"color:DodgerBlue;\">Stopped</strong>");
    }

}
