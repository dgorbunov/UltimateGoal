package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Controller;
import org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController;
import org.firstinspires.ftc.teamcode.robot.mech.HubController;
import org.firstinspires.ftc.teamcode.robot.mech.IntakeController;
import org.firstinspires.ftc.teamcode.robot.mech.ShooterController;
import org.openftc.revextensions2.ExpansionHubEx;

import java.lang.reflect.Constructor;
import java.sql.Array;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name="mainTele", group="Iterative Opmode")
public class MainTele extends OpMode {

    boolean noChassis = false;

    private DrivetrainController drive;
    private IntakeController intake;
    private ShooterController shooter;
    private HubController hub;

    private List<Controller> controllers = new ArrayList<Controller>();



    public void init() {
        telemetry.addLine("Initializing...");

        if (noChassis) {
            //TODO: implement mockDcMotor
        } else {
            drive = new DrivetrainController(hardwareMap, telemetry);
            hub = new HubController(hardwareMap);
        }


        controllers.add(drive);
        controllers.add(hub);
        for (Controller c:controllers) {
            c.init();
        }

        telemetry.clear();
        telemetry.addLine("Initialized");

    }

    public void init_loop() {
    }

    public void start() { //code to run once when play is hit
    }

    public void loop() {

        drive.drive(gamepad1);
        //shooter.shoot(gamepad1, gamepad1.a);

        telemetry.addLine(hub.getFormattedCurrentDraw());
    }

    public void stop() { //code to run when program is stopped
        telemetry.addLine("Stopping...");
        for (Controller c:controllers) {
            c.stop();
        }
        telemetry.clear();
        telemetry.addLine("Stopped");
    }

//    public static Telemetry GetTelemetry() {
//        return sTelemetry;
//    }
}
