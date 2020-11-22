package org.firstinspires.ftc.teamcode.opmodes.auto.sequence;

import android.renderscript.ScriptIntrinsicYuvToRGB;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.auto.FieldConstants;
import org.firstinspires.ftc.teamcode.robot.Controller;
import org.firstinspires.ftc.teamcode.robot.drive.SampleMecanumDrive;

public class Sequence extends Thread {

    protected Telemetry telemetry;
    protected SampleMecanumDrive drive;
    protected final static Object lock1 = new Object();

    public Sequence(HardwareMap hwMap, Telemetry tel){
        this.drive = new SampleMecanumDrive(hwMap);
        this.telemetry = tel;

        init();
    }

    protected void init() {
        synchronized (lock1) {
            if (!this.isAlive()) {
                telemetry.addData("Sequence", "start thread");
                this.start();
                this.setName("Sequence");
            }

            // Define our start pose
            // This assumes we start at x: 15, y: 10, heading: 180 degrees
            Pose2d startPose = new Pose2d(FieldConstants.StartingPosX, FieldConstants.StartingPosY, Math.toRadians(180));
            drive.setPoseEstimate(startPose);
        }
    }

    public boolean execute() throws InterruptedException {
        if (!isAlive()) {
            telemetry.addData("Sequence", "thread is not alive");
            return false;
        }

        return true;
    }

    public void wait(int millisec) {
        try {
            Thread.sleep(millisec);
        }
        catch(InterruptedException e) {
            telemetry.addData("Sequence", "Wait interrupted: %s", e.getMessage());
        }
    }

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
