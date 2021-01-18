package org.firstinspires.ftc.teamcode.opmodes.tele.params;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.opmodes.tele.Tele;

@Config //for FTCDash
public class GamepadMappings {

    Gamepad gamepad1;
    Gamepad gamepad2;

    public void setGamepads(Gamepad gamepad1, Gamepad gamepad2){
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public boolean DriveMode() {
        return gamepad1.right_bumper;
    }
    public boolean Shoot() {
        return gamepad1.a && !gamepad1.start;
    }
    public boolean ShootMan() {
        return gamepad1.y;
    }
    public boolean StartFlywheel() {
        return gamepad1.b && !gamepad1.start;
    }
    public boolean Intake() {
        if (Tele.DriverMode == DriverMode.OneDriver) return gamepad1.back;
        return gamepad2.back;
    }
    public boolean VertIntake() {
        if (Tele.DriverMode == DriverMode.OneDriver) return gamepad1.start && !gamepad1.b && !gamepad1.a;
        return gamepad2.start && !gamepad2.b && !gamepad2.a;
    }
    public boolean StopAllIntakes() {
        if (Tele.DriverMode == DriverMode.OneDriver) return gamepad1.x;
        return gamepad2.x;
    }
    public boolean WobbleGrip() {
        if (Tele.DriverMode == DriverMode.OneDriver) return gamepad1.dpad_left || gamepad1.dpad_right;
        return gamepad2.dpad_left || gamepad2.dpad_right;
    };
    public boolean WobbleArm() {
        if (Tele.DriverMode == DriverMode.OneDriver) return gamepad1.dpad_up || gamepad1.dpad_down;
        return gamepad2.dpad_up || gamepad2.dpad_down;
    };

    public enum DriverMode {
        OneDriver, TwoDrivers
    }
}