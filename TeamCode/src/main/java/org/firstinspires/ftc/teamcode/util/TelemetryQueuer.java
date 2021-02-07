package org.firstinspires.ftc.teamcode.util;

import android.util.Log;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.robot.Controller;

public class TelemetryQueuer implements Controller {

    //TODO: extends multipletelemetry, controller?
    //https://stackoverflow.com/questions/9754076/which-html-tags-are-supported-by-android-textview

    /**
     * Queues telemetry to be sent out on one call
     * Prevents multiple threads from using telemetry
     */

    //TODO: finish, implement telemetry. Refactor all telemetry to this class?

    private MultipleTelemetry telemetry;

    public TelemetryQueuer(MultipleTelemetry telemetry){
        this.telemetry = telemetry;
    }

    public MultipleTelemetry getTelemetry() {
        return telemetry;
    }

    public void addData(String info){
        telemetry.addLine(info);
        Log.i("telemetry: ", info);
    }

    public void addLine() {

    }

    public void send() {
        telemetry.update();
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
