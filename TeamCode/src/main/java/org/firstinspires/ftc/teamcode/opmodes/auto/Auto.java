package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.opmodes.OpModeBase;
import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;
import org.firstinspires.ftc.teamcode.opmodes.auto.sequence.BlueLeftSequence;
import org.firstinspires.ftc.teamcode.opmodes.auto.sequence.BlueRightSequence;
import org.firstinspires.ftc.teamcode.opmodes.auto.sequence.RedLeftSequence;
import org.firstinspires.ftc.teamcode.opmodes.auto.sequence.RedRightSequence;
import org.firstinspires.ftc.teamcode.opmodes.auto.sequence.Sequence;
import org.firstinspires.ftc.teamcode.robot.ControllerManager;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

@Disabled // will not show up on driver station
@Autonomous(name="Auto", group="Iterative Opmode")
public class Auto extends OpModeBase {

    // Maps case-insensitive name to a sequence
    private Map<String, Sequence> sequences = new HashMap<>();
    private ControllerManager controllers;
    private int ringCount = -1;
    private Sequence currentSequence;
    protected final static Object lock = new Object();
    protected String sequenceName;
    public static FieldConstants.Side side;
    public static FieldConstants.Alliance alliance;

    @Override
    public void init() {
        super.init(OPMODE.Auto);

        makeSequences();
        synchronized (lock) {
            currentSequence = getSequence(sequenceName);
            if (currentSequence == null) {
                multiTelemetry.addLine("No sequence found!");
                multiTelemetry.update();
                return;
            }
        }

    }

    @Override
    public void init_loop() {
        getRingCount();
    }

    @Override
    public void start() { //code to run once when play is hit
        super.start();

            try {
                multiTelemetry.addLine("Initializing sequence: " + getSequenceName(currentSequence));
                currentSequence.init(ringCount);

                multiTelemetry.addLine("Executing sequence: " + getSequenceName(currentSequence));
                multiTelemetry.update();

                // execute runs async
                currentSequence.execute();
            } catch (Exception e) {
                multiTelemetry.addLine("Exception while executing sequence: " + e.toString());
                multiTelemetry.update();
            }
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        synchronized (lock) {
            if (currentSequence != null) currentSequence.stop();
            super.stop();
        }
    }

    public String getSequence() {
        synchronized (lock) {
            return this.sequenceName;
        }
    }

    public void SetSequenceName(String sequenceName) {
        synchronized (lock) {
            this.sequenceName = sequenceName;
        }
    }

    @Deprecated
    protected String makeSequenceName(String alliance, String side) {
        return (alliance + side).toLowerCase();
    }

    protected String makeSequenceName(FieldConstants.Alliance alliance, FieldConstants.Side side) {
        return (alliance.toString() + side.toString()).toLowerCase();
    }

    public static String getSide() {
        return side.toString();
    }

    public static FieldConstants.Alliance getAlliance() {
        return alliance;
    }

    private void makeSequences() {
        synchronized (lock) {
            sequences.put(makeSequenceName(
                    FieldConstants.Alliance.Red, FieldConstants.Side.Left),
                    new RedLeftSequence(controllers, multiTelemetry));

            sequences.put(makeSequenceName(
                    FieldConstants.Alliance.Red, FieldConstants.Side.Right),
                    new RedRightSequence(controllers, multiTelemetry));

            sequences.put(makeSequenceName(
                    FieldConstants.Alliance.Blue, FieldConstants.Side.Left),
                    new BlueLeftSequence(controllers, multiTelemetry));

            sequences.put(makeSequenceName(
                    FieldConstants.Alliance.Blue, FieldConstants.Side.Right),
                    new BlueRightSequence(controllers, multiTelemetry));
        }
    }

    private Sequence getSequence(String name) {
        return sequences.get(name.toLowerCase());
    }

    private String getSequenceName(Sequence sequence){
        return sequence.getClass().getSimpleName();
    }

    private void getRingCount() {
        if (camera == null) {
            multiTelemetry.addLine("Camera not initialized");
            multiTelemetry.update();
            return;
        }

        multiTelemetry.addData("Camera returned rings", camera.getRingCountStr());

        int rings = camera.getRingCount();
        synchronized (lock) {
            ringCount = Optional.ofNullable(rings).orElse(-1); //if null return -1
            multiTelemetry.addData("Camera returned rings", ringCount);
            multiTelemetry.update();
        }
    }
}
