package org.firstinspires.ftc.teamcode.opmodes.tele.base;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

@Disabled
public class TeleConstants{

    static Gamepad gamepad1;
    static Gamepad gamepad2;

    public TeleConstants(Gamepad gamepad1, Gamepad gamepad2){
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public static final boolean Shoot = gamepad1.a;
    public static final boolean StartFlywheel = gamepad1.b;
    public static final boolean IntakeForward = gamepad1.dpad_right;
    public static final boolean IntakeReverse = gamepad1.dpad_left;
    public static final boolean VertIntakeForward = gamepad1.dpad_up;
    public static final boolean VertIntakeReverse = gamepad1.dpad_down;
    public static final boolean StopAllIntakes = gamepad1.x;

}
