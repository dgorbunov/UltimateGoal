package org.firstinspires.ftc.teamcode.util;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetryQueuer {

    /**
     * Manages pushing telemetry out when using multiple threads
     * All telemetry is sent out here on one thread
     */

    //TODO: finish, implement telemetry. Refactor all telemetry to this class?

    Telemetry telemetry;
    FtcDashboard dashboard;

    public TelemetryQueuer(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    public void queue(String info){
        Log.i("Telemetry: ", info);
        telemetry.addLine(info);
    }

}
