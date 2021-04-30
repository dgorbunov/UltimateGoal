package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp(name="IntakeTest", group="Iterative Opmode")
public class IntakeTest extends OpMode {

    DcMotor intake1;
    DcMotor intake2;
    public static double power1 = -0.6;
    public static double power2 = -0.4;
    boolean stopped = false;

    @Override
    public void init() {
        intake1 = hardwareMap.get(DcMotor.class, "intake");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        intake1.setPower(power1);
        intake2.setPower(power2);
    }

    @Override
    public void loop() {

    }

    public void stop() {

    }
}
