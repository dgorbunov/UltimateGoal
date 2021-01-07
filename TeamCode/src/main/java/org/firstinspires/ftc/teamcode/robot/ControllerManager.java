package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.auto.params.FieldConstants;
import org.firstinspires.ftc.teamcode.robot.camera.CameraController;
import org.firstinspires.ftc.teamcode.robot.drive.DriveLocalizationController;
import org.firstinspires.ftc.teamcode.robot.systems.BumperController;
import org.firstinspires.ftc.teamcode.robot.systems.HubController;
import org.firstinspires.ftc.teamcode.robot.systems.IntakeController;
import org.firstinspires.ftc.teamcode.robot.systems.ShooterController;
import org.firstinspires.ftc.teamcode.robot.systems.VertIntakeController;
import org.firstinspires.ftc.teamcode.robot.systems.WobbleController;

import java.util.HashMap;
import java.util.Map;

public class ControllerManager implements Controller{

    // Maps case-insensitive controllerName to a controller
    private Map<String, Controller> controllers = new HashMap<>();
    protected final Object lock = new Object();
    Telemetry telemetry;

    public ControllerManager(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void make(HardwareMap hardwareMap, Telemetry telemetry){
        add(FieldConstants.Drive, new DriveLocalizationController(hardwareMap, telemetry));
        add(FieldConstants.Hub, new HubController(hardwareMap, telemetry));
        add(FieldConstants.Intake, new IntakeController(hardwareMap, telemetry));
        add(FieldConstants.VertIntake, new VertIntakeController(hardwareMap, telemetry));
        add(FieldConstants.Shooter, new ShooterController(hardwareMap, telemetry));
        add(FieldConstants.Bumper, new BumperController(hardwareMap, telemetry));
        add(FieldConstants.Wobble, new WobbleController(hardwareMap, telemetry));
        add(FieldConstants.Camera, new CameraController(hardwareMap, telemetry));
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
