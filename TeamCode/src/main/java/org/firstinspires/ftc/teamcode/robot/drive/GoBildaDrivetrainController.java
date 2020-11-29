package org.firstinspires.ftc.teamcode.robot.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.tele.MainTele;
import org.firstinspires.ftc.teamcode.robot.Controller;

import java.util.Arrays;
import java.util.List;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

public class GoBildaDrivetrainController extends DriveLocalizationController {
    public GoBildaDrivetrainController(HardwareMap hardwareMap, Telemetry tel) {
        super(hardwareMap);
    }

    @Override
    public void init() {
        super.init();

        // GoBilda has motor mounts flipped
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        setZeroPowerBehavior(BRAKE);
    }
}
