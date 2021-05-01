package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Sleep;

@Config
@TeleOp(name="TurretTest", group="Iterative Opmode")
public class TurretTest extends OpMode {

    public static double centerPos = 0.5;
    public static double turretMaxAngle = 180;
    public static double turretLowerLimit = 0.25;
    public static double turretUpperLimit = 0.75;

    /**
     * Origin in center
     * ===========================
     * |               | 0 deg
     * |               |
     * |               |
     * |               |
     * |             _____
     * | -90 -------|  •  |---------- +90 deg
     */

    Servo turret;

    @Override
    public void init() {
        turret = hardwareMap.get(Servo.class, "side_wobble");
        turret.setPosition(centerPos);
    }

    public void turnTurretAbsolute(double deg) {
        double targetPosition = deg / turretMaxAngle + centerPos;
        if (targetPosition > turretUpperLimit) targetPosition = turretUpperLimit;
        else if (targetPosition < turretLowerLimit) targetPosition = turretLowerLimit;
        turret.setPosition(targetPosition);
    }

    @Override
    public void loop() {
        turnTurretAbsolute(0);
        Sleep.sleep(3500);
        turnTurretAbsolute(90);
        Sleep.sleep(3500);
        turnTurretAbsolute(-90);
        Sleep.sleep(3500);
    }

    public void stop() {
        turret.setPosition(centerPos);
    }
}
