package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Sleep {
    public static void sleep(int ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
    public static void sleep(double ms) {
        try {
            Thread.sleep((int)ms);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
}
