package org.firstinspires.ftc.teamcode.opmodes.auto.sequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.auto.Constants;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;
import org.firstinspires.ftc.teamcode.robot.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.systems.IntakeController;
import org.firstinspires.ftc.teamcode.robot.systems.ShooterController;
import org.firstinspires.ftc.teamcode.robot.systems.WobbleController;

public class Sequence extends Thread {

    protected int ringCount;
    protected Telemetry telemetry;
    protected SampleMecanumDrive drive;
    protected ControllerManager controllers;
    protected final static Object lock1 = new Object();

    public Sequence(int ringCount, ControllerManager controllers, HardwareMap hwMap, Telemetry tel){
        this.ringCount = ringCount;
        this.drive = new SampleMecanumDrive(hwMap);
        this.telemetry = tel;
        this.controllers = controllers;
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
        WobbleController wobble = (WobbleController)controllers.get(Constants.Wobble);
        wobble.start();
        wobble.drop();
    }

    // TODO: 11/21/2020 implement the trajectory execution
    public void moveToStart() {
    }

    // TODO: 11/21/2020 implement the trajectory execution
    public void collectWobble() {
        WobbleController wobble = (WobbleController)controllers.get(Constants.Wobble);
        wobble.start();
        wobble.pickup();
    }

    // TODO: 11/21/2020 implement the trajectory execution
    public void moveToShoot(){
    }

    // TODO: 11/21/2020 implement the shooter
    public void shootRings() {
        ShooterController shooter = (ShooterController)controllers.get(Constants.Shooter);
        shooter.start();
    }

    // TODO: 11/21/2020 implement the intake
    public void intakeRings() {
        IntakeController intake = (IntakeController)controllers.get(Constants.Intake);
        intake.start();
    }

    // TODO: 11/21/2020 implement the trajectory execution
    public void moveToLaunchLine() {
    }
}
