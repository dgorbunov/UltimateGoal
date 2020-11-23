package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class ControllerManager implements Controller{

    // Maps case-insensitive name to a controller
    private Map<String, Controller> controllers = new HashMap<String, Controller>();

    public void add(String name, Controller controller){
        controllers.put(name.toLowerCase(), controller);
    }

    public Controller get(String name){
        return controllers.get(name.toLowerCase());
    }

    @Override
    public void init() {
        for (Map.Entry entry : controllers.entrySet()) {
            Controller c = (Controller)entry.getValue();
            c.init();
        }
    }

    @Override
    public void start() {
        for (Map.Entry entry : controllers.entrySet()) {
            Controller c = (Controller)entry.getValue();
            c.start();
        }
    }

    @Override
    public void stop() {
        for (Map.Entry entry : controllers.entrySet()) {
            Controller c = (Controller)entry.getValue();
            c.stop();
        }
    }
}
