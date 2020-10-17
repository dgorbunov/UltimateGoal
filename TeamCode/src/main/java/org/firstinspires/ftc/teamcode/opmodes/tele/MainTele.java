package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Controller;
import org.firstinspires.ftc.teamcode.robot.DrivetrainController;

@TeleOp(name="mainTele", group="Iterative Opmode")
public class MainTele extends OpMode {

    DrivetrainController drive;
    Controller intake;

    public void init(boolean test){
        if (test) {
        //    drive = new mockController;
        } else drive = new DrivetrainController();
    }

    public void init_loop(){
        drive.setPower();
    }

    public void start(){ //code to run once when play is hit
        /*
        Concept class hierarchy
         */
//        if (gamepadController.macro1) {
//                drive.beginMoving();
//        } else drive.stopMoving();
//        gamepadController.intake();
    }

    public void loop(){

    }

    public void stop(){ //code to run when program is stopped

    }
}
