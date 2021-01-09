package org.firstinspires.ftc.teamcode.opmodes.tele.params;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.opmodes.tele.Tele;

@Config //for FTCDash
public class GamepadMappings {

    Gamepad gamepad1;
    Gamepad gamepad2;

    public GamepadMappings(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void setGamepads(Gamepad gamepad1, Gamepad gamepad2){
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    } //TODO: move to constructor?

    public boolean ShootButton() {
        return gamepad1.a && !gamepad1.start;
    }
    public boolean FlywheelButton() {
        return gamepad1.b && !gamepad1.start;
    }
    public boolean IntakeButton() {
        if (Tele.DriverMode == DriverMode.OneDriver) return gamepad1.back;
        return gamepad2.back;
    }
    public boolean VertIntakeButton() {
        if (Tele.DriverMode == DriverMode.OneDriver) return gamepad1.start && !gamepad1.b && !gamepad1.a;
        return gamepad2.start && !gamepad2.b && !gamepad2.a;
    }
    public boolean StopIntakesButton() {
        if (Tele.DriverMode == DriverMode.OneDriver) return gamepad1.x;
        return gamepad2.x;
    }
    public boolean WobbleButton() {
        if (Tele.DriverMode == DriverMode.OneDriver) return gamepad1.y;
        return gamepad2.y;
    };

    public enum DriverMode {
        OneDriver, TwoDrivers
    }
}