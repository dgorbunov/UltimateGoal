package org.firstinspires.ftc.teamcode.opmodes.auto.trajectories;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Controller;

public class TrajectoryController implements Controller {

    public enum trajectoryList {
        redLeftFull,
        redRightFull,
        blueLeftFull,
        blueRightFull,
    };

    public TrajectoryController(Telemetry telemetry){

    }

    public void selectTrajectory(trajectoryList trajectory){

        buildTrajectory(trajectory);
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
