package org.firstinspires.ftc.teamcode.robot.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Controller;
import org.firstinspires.ftc.teamcode.util.Sleep;
import org.openftc.revextensions2.ExpansionHubEx;

import static org.firstinspires.ftc.teamcode.util.Sleep.sleep;

public class HubController implements Controller {
    private ExpansionHubEx controlHub;
    private ExpansionHubEx expansionHub;
    boolean twoHubs = false;
    private Telemetry telemetry;
    public static String ControllerName;

    int[] statusColor = {75,255,0};
    int[] errorColor = {255,0,0};
    private int delay = 20;
    private int step = 15;

    // Thread initThread = new Thread(this::initFade);
    BlinkThread blinkThread;

    class BlinkThread extends Thread {
        private long delayInMs;
        private volatile boolean shoudRun = true;
        BlinkThread(long delayInMs) {
            this.delayInMs = delayInMs;
        }

        public void stopRunning() {
            shoudRun = false;
        }

        public void run() {
            int[] noColor = new int[] {0,0,0};
            while (shoudRun) {
                setColor(statusColor, controlHub, expansionHub);
                Sleep.sleep(delayInMs);
                setColor(noColor, controlHub, expansionHub);
                Sleep.sleep(delayInMs);
            }
        }
    }

    public HubController (HardwareMap hardwareMap, Telemetry telemetry, boolean twoHubs){
        this.twoHubs = twoHubs;
        controlHub = hardwareMap.get(ExpansionHubEx.class, "Control Hub");
        if (twoHubs) expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub");

        this.telemetry = telemetry;
        ControllerName = getClass().getSimpleName();

        blinkThread = new BlinkThread(800);
    }

    public HubController (HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, true);
    }

    public String getFormattedCurrentDraw(){
        return "Using " + getCurrentDraw() + " A";
    }

    public double getCurrentDraw(){
        if (twoHubs) return expansionHub.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) + controlHub.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS);
        return controlHub.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS);
    }

    @Override
    public void init() {
//        initThread.start();
        blinkThread.start();
    }

    @Override
    public void start() {
//        initThread.interrupt();
        blinkThread.stopRunning();
        blinkThread = null;
//        statusThread.start();
    }

    @Override
    public void stop() {
//        if (initBlinkThread.isAlive()) initBlinkThread.interrupt();
//        if (statusThread.isAlive()) statusThread.interrupt();


//        setColor(errorColor, controlHub, expansionHub);
    }

    private void statusBlink() {
        while (true) {
            setColor(statusColor, controlHub, expansionHub);
            sleep(800);
            setColor(new int[] {0,0,0}, controlHub, expansionHub);
            sleep(800);
        }
    }

    private void initBlink() {
        while (true) {
            setColor(statusColor, controlHub, expansionHub);
            sleep(800);
            setColor(new int[] {0,0,0}, controlHub, expansionHub);
            sleep(800);
        }
    }

    private void initFade(){
//        while (!initThread.isInterrupted()) { //or while (true)?
//            fadeImpl(new int[] {0,0,0}, new int[] {255,255,255}, step, delay);
//            sleep(500);
//            fadeImpl(new int[] {255,255,255}, new int[] {0,0,0}, step, delay);
//            sleep(500);
//        }
    }

    private void fadeImpl(int[] current, int[] target, double step, int delay) {
        if (target.length == 3 && current.length == 3) {

            double R = current[0];
            double G = current[1];
            double B = current[2];

            double dR = (target[0] - R) / step;
            double dG = (target[1] - G) / step;
            double dB = (target[2] - B) / step;

            for (int i = 0; i < step; i++) {
                R = Math.floor(R + dR); //to prevent overflow
                G = Math.floor(G + dG);
                B = Math.floor(B + dB);
                setColor(new int[]{(int)R, (int)G, (int)B}, controlHub, expansionHub);
                sleep(delay);
            }
        }
    }

    private void setColor(ExpansionHubEx hub, int[] color) {
        if (color.length == 3) hub.setLedColor(color[0], color[1], color[2]);
    }

    private void setColor(int[] color, ExpansionHubEx... hubs) {
        if (color.length == 3) {
            for (ExpansionHubEx hub : hubs) {
                hub.setLedColor(color[0], color[1], color[2]);
            }
        }
    }
}
