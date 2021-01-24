package org.firstinspires.ftc.teamcode.util;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

public class TelemetryQueuer {

    /**
     * Manages pushing telemetry out when using multiple threads
     * All telemetry is sent out here on one thread
     */

    //TODO: finish, implement telemewtry. Refactor all telemetry to this class?

    private MultipleTelemetry telemetry;
    private FtcDashboard dashboard;

    public TelemetryQueuer(MultipleTelemetry telemetry){
        this.telemetry = telemetry;
    }

    public void queue(String info){
        Log.i("Telemetry: ", info);
        telemetry.addLine(info);
    }

}
