package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.auto.sequence.ExampleSequence;
import org.firstinspires.ftc.teamcode.opmodes.auto.sequence.RedLeftQuadSequnce;
import org.firstinspires.ftc.teamcode.opmodes.auto.sequence.Sequence;
import org.firstinspires.ftc.teamcode.robot.Controller;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class SequenceController implements Controller {

    Telemetry telemetry;
    HardwareMap hardwareMap;

    /**
     * Create all trajectories here
     */

    private List<Sequence> trajectories = new ArrayList<Sequence>();

    public SequenceController(HardwareMap hwMap, Telemetry tel){
        hardwareMap = hwMap;
        telemetry = tel;

        /**
         * Add all trajectories here
         */
        addTrajectory(
                new ExampleSequence(hwMap, tel),
                new RedLeftQuadSequnce(hwMap, tel)
        );
    }

    public Sequence selectTrajectory(String alliance, String side, String strRings, int numRings){
        telemetry.addData("Inputs: " + alliance + ", " + side + ", Rings", numRings);
        String trajectoryName = alliance + side + strRings;
        telemetry.addLine("Looking for Trajectory: " + trajectoryName);
        telemetry.addLine("Looking through " + trajectories.size() + " trajectories");

        Sequence sequence = null;
        try {
            for (Sequence t: trajectories){
                String indexName = t.getClass().getSimpleName();
                if (indexName.equals(trajectoryName)) {
                    telemetry.addLine("Found trajectory: " + indexName);
                    sequence = t;
                    break;
                }
            }
        } catch (Exception e){
            telemetry.addLine("Exception while selecting Trajectory: " + e.toString());
        }

        if (sequence == null) {
            telemetry.addLine("No Trajectory Found");
            return null;
        }

        //TODO:  trajectory Table (array with columns for alliance, side, etc.)?

        return sequence;
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
