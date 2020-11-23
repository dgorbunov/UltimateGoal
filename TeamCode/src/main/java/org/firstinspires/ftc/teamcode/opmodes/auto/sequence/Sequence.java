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
import org.firstinspires.ftc.teamcode.robot.mech.IntakeController;
import org.firstinspires.ftc.teamcode.robot.mech.ShooterController;
import org.firstinspires.ftc.teamcode.robot.mech.WobbleController;

public class Sequence extends Thread {

    protected Telemetry telemetry;
    protected SampleMecanumDrive drive;
    protected ShooterController shooter;
    protected IntakeController intake;
    protected WobbleController wobble;

    protected final static Object lock1 = new Object();

    public Sequence(HardwareMap hwMap, Telemetry tel){
        this.drive = new SampleMecanumDrive(hwMap);
        this.shooter = new ShooterController(hwMap, tel);
        this.intake = new IntakeController(hwMap, tel);
        this.wobble = new WobbleController(hwMap, tel);
        this.telemetry = tel;
    }

    public void init() {
        init(new Pose2d(0,0,0));
    }

    public void init(Pose2d startPose) {
        synchronized (lock1) {
            if (!this.isAlive()) {
                telemetry.addData("Sequence", "start thread");
                this.start();
                this.setName("Sequence");
            }

            // Define our start pose
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

    public Pose2d GetCurrentPose() {
        return drive.getPoseEstimate();
    }

    // TODO: 11/21/2020 implement the trajectory execution
    public void moveToSquares() {
        Trajectory mySequence = drive.trajectoryBuilder(GetCurrentPose())
                .forward(5)
                .build();

        drive.followTrajectory(mySequence);
    }

    // TODO: 11/21/2020 implement the trajectory execution
    public void dropWobble(){
    }

    // TODO: 11/21/2020 implement the trajectory execution
    public void moveToStart() {
    }

    // TODO: 11/21/2020 implement the trajectory execution
    public void collectWobble() {
    }

    // TODO: 11/21/2020 implement the trajectory execution
    public void moveToShoot(){
    }

    // TODO: 11/21/2020 implement the shooter
    public void shootRings() {
    }

    // TODO: 11/21/2020 implement the trajectory execution
    public void intakeRings() {
    }

    // TODO: 11/21/2020 implement the trajectory execution
    public void moveToLaunchLine() {
    }
}
