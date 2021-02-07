package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.robot.Controller;

public class TelemetryQueuer implements Controller {

    //TODO: extends multipletelemetry, controller?
    //https://stackoverflow.com/questions/9754076/which-html-tags-are-supported-by-android-textview

    /**
     * Manages pushing telemetry out when using multiple threads
     * All telemetry is sent out here on one thread
     */

    //TODO: finish, implement telemetry. Refactor all telemetry to this class?

    private MultipleTelemetry telemetry;

    public TelemetryQueuer(MultipleTelemetry telemetry){
        this.telemetry = telemetry;
    }

    public MultipleTelemetry getTelemetry() {
        return telemetry;
    }

    public void queue(String info){
        telemetry.addLine(info);
        android.util.Log.i("telemetry: ", info);
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
