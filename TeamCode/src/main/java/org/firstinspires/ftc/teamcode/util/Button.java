package org.firstinspires.ftc.teamcode.util;

public class Button {
    boolean pressed;
    int index;

    public Button() {
        initButton();
    }

    /**
     * Toggles between arbitrary number of methods every time button is depressed and pressed
     */

    public void toggle(boolean buttonState, Runnable... methods){
        if (buttonState) {
            if (!pressed) {
                pressed = true;
                Thread thread = new Thread(methods[index]);
                thread.start();
                index ++;
                if (index == methods.length) {
                    index = 0;
                }
            }
        } else pressed = false;
    }

    /**
     * Invokes selected method every call, change selected method with button
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

        methods[index].run();
    }

    /**
     * Runs method once, no matter the length of button press
     */
    public void runOnce(boolean buttonState, Runnable method){
        if (buttonState) {
            if (!pressed) {
                pressed = true;
                Thread thread = new Thread(method);
                thread.start();
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
