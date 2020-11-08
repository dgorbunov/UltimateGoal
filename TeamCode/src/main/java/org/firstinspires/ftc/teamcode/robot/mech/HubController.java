package org.firstinspires.ftc.teamcode.robot.mech;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.Controller;
import org.openftc.revextensions2.ExpansionHubEx;

public class HubController implements Controller {

    ExpansionHubEx controlHub;

    public HubController (HardwareMap hardwareMap){
        controlHub = hardwareMap.get(ExpansionHubEx.class, "Control Hub");
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
