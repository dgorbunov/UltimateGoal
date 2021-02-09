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

    int[] statusColor = {255,0,200};
    int[] initColor = {255,255,0};
    int[] errorColor = {255,0,0};
    int[] noColor = new int[] {0,0,0};

    public enum Mode {
        INIT, STATUS, ERROR
    }

    HubStatusThread hubStatusThread;

    class HubStatusThread extends Thread {
        private double sleepDelay;
        private int fadeDelay = 20;
        private int step = 15;
        private volatile boolean shouldRun = true;
        private Mode mode = Mode.STATUS;

        HubStatusThread(double sleepDelay) {
            this.sleepDelay = sleepDelay;
        }

        public void stopRunning() {
            shouldRun = false;
        }

        public void run() {
            while (shouldRun) {
                if (mode == Mode.STATUS) {
                    setColor(statusColor, controlHub, expansionHub);
                    Sleep.sleep(sleepDelay);
                    setColor(noColor, controlHub, expansionHub);
                    Sleep.sleep(sleepDelay);

                } else if (mode == Mode.INIT) {
                    fadeImpl(noColor, initColor, step, fadeDelay);
                    Sleep.sleep(sleepDelay);
                    fadeImpl(initColor, noColor, step, fadeDelay);
                    Sleep.sleep(sleepDelay);
                }
                else if (mode == Mode.ERROR) {
                    setColor(errorColor, controlHub, expansionHub);
                }
            }

        }
    }

    public HubController (HardwareMap hardwareMap, Telemetry telemetry, boolean twoHubs){
        this.twoHubs = twoHubs;
        controlHub = hardwareMap.get(ExpansionHubEx.class, "Control Hub");
        if (twoHubs) expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub");

        this.telemetry = telemetry;
        ControllerName = getClass().getSimpleName();
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
        hubStatusThread = new HubStatusThread(750);
        hubStatusThread.mode = Mode.INIT;
        hubStatusThread.start();
    }

    @Override
    public void start() {
        hubStatusThread.mode = Mode.STATUS;
    }

    @Override
    public void stop() {
        hubStatusThread.mode = Mode.ERROR;
        hubStatusThread.stopRunning();
        hubStatusThread = null;
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
