package org.firstinspires.ftc.teamcode.util;

public class Button {

    boolean buttonState;
    boolean pressed;
    int index;

    public Button() {
        reset();
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
        reset();
    }

    private void reset() {
        pressed = false;
        index = 0;
    }

}
