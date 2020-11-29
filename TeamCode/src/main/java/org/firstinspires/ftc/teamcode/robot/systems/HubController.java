package org.firstinspires.ftc.teamcode.robot.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Controller;
import org.openftc.revextensions2.ExpansionHubEx;

public class HubController implements Controller {
    private ExpansionHubEx controlHub;
    private Telemetry telemetry;
    public static String ControllerName;

    public HubController (HardwareMap hardwareMap, Telemetry telemetry){
        controlHub = hardwareMap.get(ExpansionHubEx.class, "Control Hub");
        this.telemetry = telemetry;
        ControllerName = getClass().getSimpleName();
    }

    public String getFormattedCurrentDraw(){
        return "Using " + controlHub.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) + "A";
    }

    public double getCurrentDraw(){
        return controlHub.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS);
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
