package org.firstinspires.ftc.teamcode.robot.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.tele.MainTele;
import org.firstinspires.ftc.teamcode.robot.Controller;

public class DrivetrainController implements Controller {

    DcMotor leftFront;
    DcMotor leftRear;
    DcMotor rightFront;
    DcMotor rightRear;

    Telemetry telemetry = MainTele.GetTelemetry();


    public DrivetrainController(DcMotor left_front ,
                                DcMotor left_rear,
                                DcMotor right_front,
                                DcMotor right_rear,
                                HardwareMap hardwareMap) {


    }

    private void defGoBilda(){ //GoBilda has motor mounts flipped
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
    }

    public void drive(Gamepad gamepad){

        if (!gamepad.atRest()) { //applies to analog sticks

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
                    Math.abs(rightFrontPower) > 1 || Math.abs(rightRearPower) > 1) {
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

            setPower(leftFrontPower, leftRearPower, rightFrontPower, rightRearPower);
        }
    };

    public void setPower(double LF, double LR, double RF, double RR){
        leftFront.setPower(LF);
        leftRear.setPower(LR);
        rightFront.setPower(RF);
        rightRear.setPower(RR);
    }

    private void setAllPower(double power){
        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(power);
    }

    @Override
    public void stop() {
        setAllPower(0);
    }


    @Override
    public void init() {
        defGoBilda();
    }

    @Override
    public void start() {

    }

}
