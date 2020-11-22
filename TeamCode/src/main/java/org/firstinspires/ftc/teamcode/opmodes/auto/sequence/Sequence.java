package org.firstinspires.ftc.teamcode.opmodes.auto.sequence;

import android.renderscript.ScriptIntrinsicYuvToRGB;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Controller;
import org.firstinspires.ftc.teamcode.robot.drive.SampleMecanumDrive;

public abstract class Sequence implements Controller {

    protected Telemetry telemetry;
    protected SampleMecanumDrive drive;

    public Sequence(HardwareMap hwMap, Telemetry tel){
        this.drive = new SampleMecanumDrive(hwMap);
        this.telemetry = tel;
    }

    public void init () {

    }

    public void start() {
        // Set inital pose
        Pose2d startPose = GetCurrentPose();
        drive.setPoseEstimate(startPose);
    }

    public void stop() {
    }

    public abstract void run() throws InterruptedException;

    public Pose2d GetCurrentPose()
    {
        return drive.getPoseEstimate();
    }

    // TODO: 11/21/2020 implement the trajectory execution
    public void moveToSquares(Pose2d startPos) {
        Trajectory mySequence = drive.trajectoryBuilder(startPos)
                .strafeRight(10)
                .forward(5)
                .build();

        drive.followTrajectory(mySequence);
    }

    // TODO: 11/21/2020 implement the trajectory execution
    public void dropWobble(Pose2d startPos){
    }

    // TODO: 11/21/2020 implement the trajectory execution
    public void moveToStart(Pose2d startPos) {
    }

    // TODO: 11/21/2020 implement the trajectory execution
    public void collectWobble(Pose2d startPos) {
    }

    // TODO: 11/21/2020 implement the trajectory execution
    public void moveToShoot(Pose2d startPos){
    }

    // TODO: 11/21/2020 implement the shooter
    public void shootRings() {
    }

    // TODO: 11/21/2020 implement the trajectory execution
    public void intakeRings(Pose2d startPos) {
    }

    // TODO: 11/21/2020 implement the trajectory execution
    public void moveToLaunchLine(Pose2d startPos) {
    }
}
