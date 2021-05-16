package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.OpModeBase;
import org.firstinspires.ftc.teamcode.opmodes.tele.params.MechConstants;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;
import org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController;

@TeleOp(group = "drive")
public class DrivetrainTest extends OpModeBase {

    DrivetrainController drive;

    ControllerManager controllers;

    public DrivetrainTest() { super(); }

        /**
         * User defined init method
         * <p>
         * This method will be called once when the INIT button is pressed.
         */
    @Override
    public void init() {
        OPMODE_TYPE = OPMODE.Tele;
        super.init();

        drive.setPoseEstimate(MechConstants.TeleStartingPose);

    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    @Override
    public void loop() {
        super.loop();
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );

        drive.update();

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();

    }

    @Override
    public void stop() {
        super.stop();
    }
}
