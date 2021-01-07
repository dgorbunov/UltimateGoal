package org.firstinspires.ftc.teamcode.opmodes.tele.params;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.opmodes.tele.Tele;

@Config //for FTCDash
public class TeleConstants{

    static Gamepad gamepad1;
    static Gamepad gamepad2;

    public static void setGamepads(Gamepad gamepad1, Gamepad gamepad2) {
        TeleConstants.gamepad1 = gamepad1;

        if (Tele.DriverMode == DriverMode.OneDriver) {
            //assuming key mappings do not intersect
            TeleConstants.gamepad2 = gamepad1;
        } else TeleConstants.gamepad2 = gamepad2;
    }

    public static final boolean Shoot = gamepad1.a;
    public static final boolean StartFlywheel = gamepad1.b;
    public static final boolean IntakeForward = gamepad2.dpad_right;
    public static final boolean IntakeReverse = gamepad2.dpad_left;
    public static final boolean VertIntakeForward = gamepad2.dpad_up;
    public static final boolean VertIntakeReverse = gamepad2.dpad_down;
    public static final boolean StopAllIntakes = gamepad2.x;
    public static final boolean PickupWobble = gamepad1.left_bumper;
    public static final boolean DropWobble = gamepad1.right_bumper;

    public static final double IntakeForwardPower = 0.8;
    public static final double IntakeReversePower = -0.8;
    public static final double VertIntakeForwardPower = 1;
    public static final double VertIntakeReversePower = -1;

    public enum DriverMode {
        OneDriver, TwoDrivers
    }
}