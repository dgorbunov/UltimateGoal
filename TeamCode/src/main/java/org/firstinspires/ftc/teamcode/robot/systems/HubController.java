package org.firstinspires.ftc.teamcode.robot.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Controller;
import org.openftc.revextensions2.ExpansionHubEx;

public class HubController implements Controller {
    private ExpansionHubEx controlHub;
    private ExpansionHubEx expansionHub;
    boolean twoHubs = false;
    private Telemetry telemetry;
    public static String ControllerName;

    int[] primaryHubColor = {255,255,0};
//    Thread fadeThread = new Thread(this::fade);

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
        setColor(controlHub, primaryHubColor);
        setColor(expansionHub, primaryHubColor);
    }

    @Override
    public void start() {
    }

    @Override
    public void stop() {
    }

    private void setColor(ExpansionHubEx hub, int[] color) {
        if (color.length == 3) hub.setLedColor(color[0], color[1], color[2]);
    }

    private void fade() {
//        while(!fadeThread.isInterrupted()) {
//            for (int i = 0; i < 256; i++) {
//                int[] color = {0, 0, 0};
//                for (int n = 0; n < 3; n++) {
//                    if (i < primaryHubColor[n]) color[n] = i;
//                }
//
//                setColor(controlHub, color);
//                setColor(expansionHub, color);
//                sleep(12);
//            }
//
//            for (int i = 255; i >= 0; i--) {
//                int[] color = primaryHubColor;
//                for (int n = 0; n < 3; n++) {
//                    if (i < primaryHubColor[n]) color[n] = i;
//                }
//
//                setColor(controlHub, color);
//                setColor(expansionHub, color);
//                sleep(12);
//            }
//            sleep(250);
//        }
//    }
    }
}
