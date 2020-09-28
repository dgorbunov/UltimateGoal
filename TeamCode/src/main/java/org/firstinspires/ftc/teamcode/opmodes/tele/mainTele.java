package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="mainTele", group="Iterative Opmode")
public class mainTele extends OpMode {

    DcMotor leftMotor;

    public void init(){
        leftMotor = hardwareMap.dcMotor.get("left");
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void init_loop(){
    }

    public void start(){ //code to run once when play is hit
    }

    public void loop(){
        telemetry.addData("encoder", leftMotor.getCurrentPosition() + "  busy=" + leftMotor.isBusy());
        telemetry.update();

    }

    public void stop(){ //code to run when program is stopped

    }
}
