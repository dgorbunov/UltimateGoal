package org.firstinspires.ftc.teamcode.util;

public class Sleep {

    /**
     * Wraps Thread.sleep into a convenient, static package
     */

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
