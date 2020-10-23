package org.firstinspires.ftc.teamcode.robot.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.tele.MainTele;
import org.firstinspires.ftc.teamcode.robot.Controller;

public class DrivetrainController implements Controller {

    //TODO: Implement DT controller, seperate DT controller for roadrunner trajectories?

    Controller controllers;

    DcMotor leftFront;
    DcMotor leftRear;
    DcMotor rightFront;
    DcMotor rightRear;


    public DrivetrainController(DcMotor left_front ,
                                DcMotor left_rear,
                                DcMotor right_front,
                                DcMotor right_rear,
                                HardwareMap hardwareMap) {

    }

    public void defGoBilda(){ //GoBilda has motor mounts flipped
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
    }

    public void drive(Gamepad gamepad){

        double y = -gamepad.left_stick_y; //reversed
        double x = gamepad.left_stick_x * 1.5; // Counteract imperfect strafing
        double rx = -gamepad.right_stick_x; //Might need to change for Acto

        double leftFrontPower = y + x + rx;
        double leftRearPower = y - x + rx;
        double rightFrontPower = y - x - rx;
        double rightRearPower = y + x - rx;

        // Put powers in the range of -1 to 1 only if they aren't already (not
        // checking would cause us to always drive at full speed)

        if (Math.abs(leftFrontPower) > 1 || Math.abs(leftRearPower) > 1 ||
                Math.abs(rightFrontPower) > 1 || Math.abs(rightRearPower) > 1 ) {
            // Find the largest power
            double max = 0;
            max = Math.max(Math.abs(leftFrontPower), Math.abs(leftRearPower));
            max = Math.max(Math.abs(rightFrontPower), max);
            max = Math.max(Math.abs(rightRearPower), max);

            // Divide everything by max (it's positive so we don't need to worry
            // about signs)
            leftFrontPower /= max;
            leftRearPower /= max;
            rightFrontPower /= max;
            rightRearPower /= max;
        }

        setPower(leftFrontPower,leftRearPower,rightFrontPower,rightRearPower);
        MainTele.GetTelemetry().addData("SetMotorPower", 1);
    };

    public void setPower(double LF, double LR, double RF, double RR){
        leftFront.setPower(LF);
        leftRear.setPower(LR);
        rightFront.setPower(RF);
        rightRear.setPower(RR);
    }
    public void setZeroBehavior(boolean brake){
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void stop() {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

    }


    @Override
    public void init() {

    }

    @Override
    public void start() {

    }

}
