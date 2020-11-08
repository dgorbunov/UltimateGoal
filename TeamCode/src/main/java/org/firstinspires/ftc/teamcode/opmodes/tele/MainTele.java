package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Controller;
import org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController;
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

    private List<Controller> controllers = new ArrayList<Controller>();

    public static Telemetry sTelemetry;
    MainTele() {sTelemetry = telemetry;}

    ExpansionHubEx Hub;


    public void init() {
        if (noChassis) {
        //    drive = new DrivetrainController(hardwareMap.get(MockDcMotor.class, ""));
        } else {
            drive = new DrivetrainController(
                    hardwareMap.get(DcMotor.class, "left_front"),
                    hardwareMap.get(DcMotor.class, "left_rear"),
                    hardwareMap.get(DcMotor.class, "right_front"),
                    hardwareMap.get(DcMotor.class, "right_rear"),
                    hardwareMap);
            shooter = new ShooterController(hardwareMap.get(DcMotor.class, "shooter"), hardwareMap);
        }


        controllers.add(drive);
        controllers.add(intake);
        controllers.add(shooter);
        for (Controller c:controllers) {
            c.init();
        }

    }

    public void init_loop() {
    }

    public void start() { //code to run once when play is hit
    }

    public void loop() {

        drive.drive(gamepad1);
        shooter.shoot(gamepad1, gamepad1.a);

        telemetry.addLine("Using " + Hub.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) + "A");
    }

    public void stop() { //code to run when program is stopped
        telemetry.addLine("Stopping Program");
        for (Controller c:controllers) {
            c.stop();
        }
    }

    public static Telemetry GetTelemetry() {
        return sTelemetry;
    }
}
