package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Controller;
import org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController;
import org.firstinspires.ftc.teamcode.robot.mech.IntakeController;

import java.lang.reflect.Constructor;
import java.sql.Array;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name="mainTele", group="Iterative Opmode")
public class MainTele extends OpMode {

    boolean noChassis = false;

    private DrivetrainController drive;
    private IntakeController intake;
    private List<Controller> controllers = new ArrayList<Controller>();

    public static Telemetry s_telemetry;
    MainTele() {s_telemetry = telemetry;}


    public void init() {
        if (noChassis) {
        //    drive = new mockController;
        } else drive = new DrivetrainController(
                hardwareMap.get(DcMotor.class, "left_front"),
                hardwareMap.get(DcMotor.class, "left_rear"),
                hardwareMap.get(DcMotor.class, "right_front"),
                hardwareMap.get(DcMotor.class, "right_rear"),
                hardwareMap);

        drive.defGoBilda();

        //TODO: how to init all implementations of Controller?

        controllers.add(drive);
        controllers.add(intake);
        for (Controller c:controllers) {
            c.init();
        }

    }

    public void init_loop() {
        //drive.setPower();
    }

    public void start() { //code to run once when play is hit
        /*
        Concept class hierarchy
         */
//        if (gamepadController.macro1) {
//                drive.beginMoving();
//        } else drive.stopMoving();
//        gamepadController.intake();
    }

    public void loop() {

        drive.drive(gamepad1);

    }

    public void stop() { //code to run when program is stopped
        drive.stop();
    }

    public static Telemetry GetTelemetry() {
        return s_telemetry;
    }
}
