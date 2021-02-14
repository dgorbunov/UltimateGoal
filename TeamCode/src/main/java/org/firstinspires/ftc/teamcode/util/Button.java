package org.firstinspires.ftc.teamcode.util;

/**
 * Author: Daniel
 * Easy to use interface for logic operations for booleans
 * Recommended to only use one of these methods per boolean
 */

public class Button {
    boolean pressed;
    int index;

    public Button() {
        initButton();
    }

    /**
     * Toggles between running arbitrary number of methods (once) every time button is depressed and pressed
     */

    public void toggle(boolean buttonState, Runnable... methods){
        if (buttonState) {
            if (!pressed) {
                pressed = true;
                new Thread(methods[index]).start();
                index ++;
                if (index == methods.length) {
                    index = 0;
                }
            }
        } else pressed = false;
    }

    /**
     * Runs selected method every call, change selected method with button
     */
    public void toggleLoop(boolean buttonState, Runnable... methods){
        if (buttonState){
            if (!pressed) {
                pressed = true;
                index ++;
                if (index == methods.length) {
                    index = 0;
                }
            }
        } else pressed = false;

        new Thread(methods[index]).start();
    }

    /**
     * Runs method once, no matter the length of button press
     */
    public void runOnce(boolean buttonState, Runnable method){
        if (buttonState) {
            if (!pressed) {
                pressed = true;
                new Thread(method).start();
            }
        } else pressed = false;
    }

    public void resetToggle() {
        initButton();
    }

    private void initButton() {
        pressed = false;
        index = 0;
    }

}
