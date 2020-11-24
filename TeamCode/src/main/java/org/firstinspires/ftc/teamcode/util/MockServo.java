package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MockServo implements Servo {
    String name;
    Telemetry telemetry;

    public MockServo(String name, Telemetry telemetry){
        this.name = name;
        this.telemetry = telemetry;
    }
    @Override
    public ServoController getController() {
        return null;
    }

    @Override
    public int getPortNumber() {
        return 0;
    }

    @Override
    public void setDirection(Direction direction) {
        telemetry.addData(name + ": Set Direciton", direction);
    }

    @Override
    public Direction getDirection() {
        return null;
    }

    @Override
    public void setPosition(double position) {
        telemetry.addData(name + ": Set Position", position);
    }

    @Override
    public double getPosition() {
        return 0;
    }

    @Override
    public void scaleRange(double min, double max) {
        telemetry.addData(name + ": Set Scale Range", min + " , " + max);
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return null;
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}
