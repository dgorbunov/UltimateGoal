package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.teamcode.opmodes.auto.sequence.BlueLeftSequence;
import org.firstinspires.ftc.teamcode.opmodes.auto.sequence.BlueRightSequence;
import org.firstinspires.ftc.teamcode.opmodes.auto.sequence.RedLeftSequence;
import org.firstinspires.ftc.teamcode.opmodes.auto.sequence.RedRightSequence;
import org.firstinspires.ftc.teamcode.opmodes.auto.sequence.Sequence;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;
import org.firstinspires.ftc.teamcode.robot.camera.CameraController;
import org.firstinspires.ftc.teamcode.robot.systems.IntakeController;
import org.firstinspires.ftc.teamcode.robot.systems.ShooterController;
import org.firstinspires.ftc.teamcode.robot.systems.WobbleController;

import java.util.HashMap;
import java.util.Map;

@Autonomous(name="fullAuto", group="Iterative Opmode")
public class FullAuto extends OpMode {

    // Maps case-insensitive name to a sequence
    private Map<String, Sequence> sequences = new HashMap<String, Sequence>();
    private ControllerManager controllers = new ControllerManager();
    private String strNumRings;
    private Sequence currentSequence;

    @Override
    public void init() {
        telemetry.addLine("Initializing...");

        controllers.add(Constants.Camera, new CameraController(hardwareMap, telemetry));

        controllers.init();

        makeSequences();

        telemetry.addLine("Initialized");
    }

    @Override
    public void init_loop() {
        CameraController camera = (CameraController)controllers.get(Constants.Camera);
        strNumRings = camera.rankRings();
        telemetry.addLine(strNumRings);
    }

    @Override
    public void start() { //code to run once when play is hit
        controllers.start(); //stop camera instance

        // TODO: use different opmodes for alliance, side. For now, we are assuming Red Alliance, Left side:
        String sequenceName = makeSequenceName(Constants.RedAlliance, Constants.LeftSide, strNumRings);
        currentSequence = getSequence(sequenceName);

        if (currentSequence == null) {
            telemetry.addLine("No sequence found!");
            return;
        }

        try {
            telemetry.addLine("Initializing sequence: " + sequenceName);
            currentSequence.init();

            telemetry.addLine("Executing sequence: " + sequenceName);

            // execute runs async
            currentSequence.execute();
        } catch (Exception e){
            telemetry.addLine("Exception while executing sequence: " + e.toString());
        }
    }

    @Override
    public void loop() {
    }

    @Override
    public void stop() {
        currentSequence.stop();
        controllers.stop();
    }

    private void makeSequences() {

        sequences.put(makeSequenceName(
                Constants.BlueAlliance, Constants.LeftSide, Constants.NoRings),
                new BlueLeftSequence(0, controllers, hardwareMap, telemetry));

        sequences.put(makeSequenceName(
                Constants.BlueAlliance, Constants.RightSide, Constants.NoRings),
                new BlueRightSequence(0, controllers, hardwareMap, telemetry));

        sequences.put(makeSequenceName(
                Constants.BlueAlliance, Constants.LeftSide, Constants.SingleRing),
                new BlueLeftSequence(1, controllers, hardwareMap, telemetry));

        sequences.put(makeSequenceName(
                Constants.BlueAlliance, Constants.RightSide, Constants.SingleRing),
                new BlueRightSequence(1, controllers, hardwareMap, telemetry));

        sequences.put(makeSequenceName(
                Constants.BlueAlliance, Constants.LeftSide, Constants.QuadRing),
                new BlueLeftSequence(4, controllers, hardwareMap, telemetry));

        sequences.put(makeSequenceName(
                Constants.BlueAlliance, Constants.RightSide, Constants.QuadRing),
                new BlueRightSequence(4, controllers, hardwareMap, telemetry));

        sequences.put(makeSequenceName(
                Constants.RedAlliance, Constants.LeftSide, Constants.NoRings),
                new RedLeftSequence(0, controllers, hardwareMap, telemetry));

        sequences.put(makeSequenceName(
                Constants.RedAlliance, Constants.RightSide, Constants.NoRings),
                new RedRightSequence(0, controllers, hardwareMap, telemetry));

        sequences.put(makeSequenceName(
                Constants.RedAlliance, Constants.LeftSide, Constants.SingleRing),
                new RedLeftSequence(1, controllers, hardwareMap, telemetry));

        sequences.put(makeSequenceName(
                Constants.RedAlliance, Constants.RightSide, Constants.SingleRing),
                new RedRightSequence(1, controllers, hardwareMap, telemetry));

        sequences.put(makeSequenceName(
                Constants.RedAlliance, Constants.LeftSide, Constants.QuadRing),
                new RedLeftSequence(4, controllers, hardwareMap, telemetry));

        sequences.put(makeSequenceName(
                Constants.RedAlliance, Constants.RightSide, Constants.QuadRing),
                new RedRightSequence(4, controllers, hardwareMap, telemetry));
    }

    private String makeSequenceName(String alliance, String side, String rings) {
        return (alliance + side).toLowerCase();
    }

    private Sequence getSequence(String name) {
        return sequences.get(name.toLowerCase());
    }
}
