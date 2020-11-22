package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.auto.sequence.ExampleSequence;
import org.firstinspires.ftc.teamcode.opmodes.auto.sequence.SequenceFollower;
import org.firstinspires.ftc.teamcode.opmodes.auto.sequence.RedLeftQuad;
import org.firstinspires.ftc.teamcode.opmodes.auto.sequence.Sequence;
import org.firstinspires.ftc.teamcode.robot.Controller;
import org.firstinspires.ftc.teamcode.robot.drive.SampleMecanumDrive;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class SequenceController implements Controller {

    Telemetry telemetry;
    HardwareMap hardwareMap;
    SampleMecanumDrive sampleMecanumDrive;
    SequenceFollower globalTrajectory;


    /**
     * Create all trajectories here
     */
    public ExampleSequence ExampleTrajectory = new ExampleSequence();
    public RedLeftQuad RedLeftQuad = new RedLeftQuad();


    private static List<Sequence> trajectories = new ArrayList<Sequence>();


    public SequenceController(HardwareMap hwMap, Telemetry tel){
        hardwareMap = hwMap;
        telemetry = tel;
        sampleMecanumDrive = new SampleMecanumDrive(hardwareMap);
        globalTrajectory = new SequenceFollower(sampleMecanumDrive, telemetry);

        /**
         * Add all trajectories here
         */
        addTrajectory(ExampleTrajectory, RedLeftQuad);
    }

    public void selectTrajectory(String alliance, String side, String strRings, int numRings){
        telemetry.addData("Inputs: " + alliance + ", " + side + ", Rings", numRings);
        String trajectoryName = alliance + side + strRings;
        telemetry.addLine("Looking for Trajectory: " + trajectoryName);
        telemetry.addLine("Looking through " + trajectories.size() + " trajectories");
        try {
//            for (int i = 0;i < trajectories.size(); i++){
//               String t = trajectories.get(i).getClass().getSimpleName();
//               telemetry.addLine(t);
//                if (t.equals(trajectoryName)) {
//                    telemetry.addLine("Running Trajectory: " + t);
//                    runTrajectory(trajectories.get(i));
//                }
//                else if (i == trajectories.size() - 1) {
//                    telemetry.addLine("No Trajectory Found");
//                }
            for (Sequence t: trajectories){
                String indexName = t.getClass().getSimpleName();
                if (indexName.equals(trajectoryName)) {
                    telemetry.addLine("Running Trajectory: " + indexName);
                    runTrajectory(t);
                    break;
                }
                else if (t == trajectories.get(trajectories.lastIndexOf(trajectories))){
                    telemetry.addLine("No Trajectory Found");
                }
            }
        } catch (Exception e){
            telemetry.addLine(e.toString());
            telemetry.addLine("No Trajectory found");
        }

        //TODO:  trajectory Table (array with columns for alliance, side, etc.)?
    }

    public void runTrajectory(Sequence sequence) {
        sequence.run(); //hang on this
    }

    private void addTrajectory(Sequence... traj){
        trajectories.addAll(Arrays.asList(traj));
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
