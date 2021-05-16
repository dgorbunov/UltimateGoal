package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.opmodes.OpModeBase;
import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;
import org.firstinspires.ftc.teamcode.opmodes.auto.sequence.BlueLeftSequence;
import org.firstinspires.ftc.teamcode.opmodes.auto.sequence.BlueRightSequence;
import org.firstinspires.ftc.teamcode.opmodes.auto.sequence.RedLeftSequence;
import org.firstinspires.ftc.teamcode.opmodes.auto.sequence.RedRightSequence;
import org.firstinspires.ftc.teamcode.opmodes.auto.sequence.Sequence;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.Alliance.Blue;
import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.Alliance.Red;
import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.RedField.RingPos;
import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.Side.Left;
import static org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants.Side.Right;

@Disabled
@Autonomous(name="Auto", group="Iterative Opmode")
public class Auto extends OpModeBase {

    // Maps case-insensitive name to a sequence
    private Map<String, Sequence> sequences = new HashMap<>();
    private Sequence currentSequence;
    protected String sequenceName;
    protected final static Object lock = new Object();
    public static FieldConstants.Side side;
    public static FieldConstants.Alliance alliance;
    private int ringCount = -1;

    @Override
    public void init() {
        OPMODE_TYPE = OPMODE.Auto;
        super.init();

        makeSequences();
        synchronized (lock) {
            currentSequence = getSequence(sequenceName);
            if (currentSequence == null) {
                RobotLog.addGlobalWarningMessage("No sequence found");
                return;
            }
        }

    }

    @Override
    public void init_loop() {
        getRingCount();
        drive.drawRingsOverridePacket(RingPos, ringCount);
    }

    @Override
    public void start() { //code to run once when play is hit
        super.start();

            try {
                telemetryd.addLine("Initializing sequence: " + getSequenceName(currentSequence));
                currentSequence.init(ringCount);

                telemetryd.addLine("Executing sequence: " + getSequenceName(currentSequence));

                // execute runs async
                currentSequence.execute();
            } catch (Exception e) {
                RobotLog.addGlobalWarningMessage("Exception while executing sequence: " + e.toString());
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
            sequences.put(makeSequenceName(Red, Left),
                    new RedLeftSequence(controllers, telemetryd));

            sequences.put(makeSequenceName(Red, Right),
                    new RedRightSequence(controllers, telemetryd));

            sequences.put(makeSequenceName(Blue, Left),
                    new BlueLeftSequence(controllers, telemetryd));

            sequences.put(makeSequenceName(Blue, Right),
                    new BlueRightSequence(controllers, telemetryd));
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
            RobotLog.addGlobalWarningMessage("Camera not initialized");
            return;
        }

        telemetryd.addData("Camera returned rings", camera.getRingCountStr());

        int rings = camera.getRingCount();
        synchronized (lock) {
            ringCount = Optional.ofNullable(rings).orElse(-1); //if null return -1
            telemetryd.addData("Camera returned rings", ringCount);
        }
    }
}
