package org.firstinspires.ftc.teamcode.opmodes.auto.trajectories;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Controller;
import org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController;

public class TrajectoryController implements Controller {

    private DrivetrainController drive;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    public enum trajectoryList {
        redLeftFull,
        redRightFull,
        blueLeftFull,
        blueRightFull,
    };

    public TrajectoryController(HardwareMap hwMap, Telemetry tel){
        hardwareMap = hwMap;
        telemetry = tel;
        drive = new DrivetrainController(hardwareMap, telemetry);
    }

    public void selectTrajectory(String alliance, String side, int numRings){
        telemetry.addData("Selecting Trajectory: " + alliance + " " + side + " Rings", numRings);
        //TODO:  trajectory Tabel (array with columns for alliance, side, etc.)
    }

    private void buildTrajectory(trajectoryList trajectory){
    }

    public void runTrajectory(trajectoryList trajectory) {

    }

    @Override
    public void init() {

    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {

    }
}
