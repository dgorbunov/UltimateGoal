package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetryQueuer{

    /**
     * Manages pushing telemetry out when using multiple threads
     * All telemetry is sent out here on one thread
     */

    //TODO: finish, implement telemtry. Refactor all telemetry to this class?

    Telemetry telemetry;

    public TelemetryQueuer(Telemetry telemetry){
        this.telemetry = telemetry;
    }

}
