package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.ControllerManager;
import org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController;
import org.firstinspires.ftc.teamcode.robot.mech.HubController;
import org.firstinspires.ftc.teamcode.robot.mech.IntakeController;
import org.firstinspires.ftc.teamcode.robot.mech.ShooterController;

@TeleOp(name="mainTele", group="Iterative Opmode")
public class MainTele extends OpMode {

    boolean noChassis = false;

    private DrivetrainController drive;
    private IntakeController intake;
    private ShooterController shooter;
    private HubController hub;

    private ControllerManager controllers = new ControllerManager();

    public void init() {
        telemetry.addLine("Initializing...");

        if (noChassis) {
            //TODO: implement mockDcMotor
        } else {
            drive = new DrivetrainController(hardwareMap, telemetry);
            hub = new HubController(hardwareMap, telemetry);
        }

        controllers.init();

        telemetry.clear();
        telemetry.addLine("Initialized");
    }

    public void init_loop() {
    }

    public void start() { //code to run once when play is hit
        controllers.start();
    }

    public void loop() {

        drive.drive(gamepad1);
        //shooter.shoot(gamepad1, gamepad1.a);

        telemetry.addLine(hub.getFormattedCurrentDraw());
    }

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
