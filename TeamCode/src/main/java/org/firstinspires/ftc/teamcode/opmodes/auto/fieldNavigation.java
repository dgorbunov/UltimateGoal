package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController;

@Autonomous(name="fieldNavigation", group="Iterative Opmode")
public class fieldNavigation extends OpMode  {

    boolean noChassis = false;

    private DrivetrainController drive;
    private vuforiaNavTargets vuforia;

    @Override
    public void init() {
        if (noChassis) {
            //    drive = new mockController;
        } else drive = new DrivetrainController(
                hardwareMap.get(DcMotor.class, "left_front"),
                hardwareMap.get(DcMotor.class, "left_rear"),
                hardwareMap.get(DcMotor.class, "right_front"),
                hardwareMap.get(DcMotor.class, "right_rear"),
                hardwareMap, telemetry);

        drive.defGoBilda();

    }

    @Override
    public void loop() {

        vuforia.runOpMode();
        vuforia.returnHeading();

    }
}
