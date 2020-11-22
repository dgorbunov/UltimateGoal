package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public  class ControllerManager implements Controller{

    private List<Controller> controllers = new ArrayList<>();

    public ControllerManager(Controller... controllerChildren){
        addChildren(controllerChildren);
    }

    public void addChildren(Controller... controllerChildren){
        controllers.addAll(Arrays.asList(controllerChildren));
    }


    @Override
    public void init() {
        for (Controller c : controllers) c.init();

    }

    @Override
    public void start() {
        for (Controller c : controllers) c.start();

    }

    @Override
    public void stop() {
        for (Controller c : controllers) c.stop();
    }
}
