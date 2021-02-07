package org.firstinspires.ftc.teamcode.util;

import android.util.Log;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

public class TelemetryQueuer{

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
        Log.i("Telemetry: ", info);
        telemetry.addLine(info);
    }
}
