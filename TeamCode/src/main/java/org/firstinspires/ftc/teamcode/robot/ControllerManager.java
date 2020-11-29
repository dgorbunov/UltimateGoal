package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.drive.DrivetrainController;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class ControllerManager implements Controller{

    // Maps case-insensitive controllerName to a controller
    private Map<String, Controller> controllers = new HashMap<>();
    protected final Object lock = new Object();
    Telemetry telemetry;

    public ControllerManager(Telemetry tel) {
        this.telemetry = tel;
    }

    public void add(String controllerName, Controller controller) {
        synchronized (lock) {
            controllers.put(controllerName.toLowerCase(), controller);
        }
    }

    @Nullable
    public <T> T get(Class<? extends T> classOrInterface, String controllerName) {
        synchronized (lock) {
            controllerName = controllerName.trim().toLowerCase();
            T result = tryGet(classOrInterface, controllerName);
            if (result == null) {
                telemetry.addData("ControllerManager", String.format("Unable to find a controller with controllerName \"%s\" and type %s",
                            controllerName,
                            classOrInterface.getSimpleName()));
            }
            return result;
        }
    }

    @Nullable
    public <T> T tryGet(Class<? extends T> classOrInterface, String controllerName) {
        synchronized (lock) {
            controllerName = controllerName.trim().toLowerCase();
            Controller controller = controllers.get(controllerName);
            if (controller != null) {
                if (classOrInterface.isInstance(controller)) {
                    return classOrInterface.cast(controller);
                }
            }

            return null;
        }
    }

    @Nullable
    public Controller get(String controllerName){
        synchronized (lock) {
            return controllers.get(controllerName.toLowerCase());
        }
    }

    @Override
    public void init() {
        synchronized (lock) {
            telemetry.addData("ControllerManager", "init");
            for (Map.Entry<String, Controller> entry : controllers.entrySet()) {
                Controller c = (Controller) entry.getValue();
                c.init();
            }
        }
    }

    @Override
    public void start() {
        synchronized (lock) {
            telemetry.addData("ControllerManager", "start");
            for (Map.Entry<String, Controller> entry : controllers.entrySet()) {
                Controller c = (Controller) entry.getValue();
                c.start();
            }
        }
    }

    @Override
    public void stop() {
        synchronized (lock) {
            telemetry.addData("ControllerManager", "stop");
            for (Map.Entry<String, Controller> entry : controllers.entrySet()) {
                Controller c = (Controller) entry.getValue();
                c.stop();
            }
        }
    }
}
