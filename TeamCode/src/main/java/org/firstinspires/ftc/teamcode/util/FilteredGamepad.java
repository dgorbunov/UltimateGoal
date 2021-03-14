package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

public class FilteredGamepad extends Gamepad {

    private Gamepad gamepad;
    private double filterTime = 8;
    private updateInternal updateThread = new updateInternal();

    public float left_stick_x = 0f;
    public float left_stick_y = 0f;
    public float right_stick_x = 0f;
    public float right_stick_y = 0f;
    public boolean dpad_up = false;
    public boolean dpad_down = false;
    public boolean dpad_left = false;
    public boolean dpad_right = false;
    public boolean a = false;
    public boolean b = false;
    public boolean x = false;
    public boolean y = false;
    public boolean guide = false;
    public boolean start = false;
    public boolean back = false;
    public boolean left_bumper = false;
    public boolean right_bumper = false;
    public boolean left_stick_button = false;
    public boolean right_stick_button = false;
    public float left_trigger = 0f;
    public float right_trigger = 0f;
    public boolean circle = false;
    public boolean cross = false;
    public boolean triangle = false;
    public boolean square = false;
    public boolean share = false;
    public boolean options = false;
    public boolean touchpad = false;
    public boolean ps = false;

    public FilteredGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public FilteredGamepad(Gamepad gamepad, int filterTime) {
        this.gamepad = gamepad;
        this.filterTime = filterTime;
    }

    public void start() {
        updateThread.start();
    }

    public void stop() {
        updateThread.interrupt();
    }

    class updateInternal extends Thread {
        public void run() {
            while (true) {
                update();
                Sleep.sleep(filterTime);
            }
        }
    }

    private void update() {
        left_stick_x = gamepad.left_stick_x;
        left_stick_y = gamepad.left_stick_y;
        right_stick_x = gamepad.right_stick_x;
        right_stick_y = gamepad.right_stick_y;
        dpad_up = gamepad.dpad_up;
        dpad_down = gamepad.dpad_down;
        dpad_left = gamepad.dpad_left;
        dpad_right = gamepad.dpad_right;
        a = gamepad.a;
        b = gamepad.b;
        x = gamepad.x;
        y = gamepad.y;
        guide = gamepad.guide;
        start = gamepad.start;
        back = gamepad.back;
        left_bumper = gamepad.left_bumper;
        right_bumper = gamepad.right_bumper;
        left_stick_button = gamepad.left_stick_button;
        right_stick_button = gamepad.right_stick_button;
        left_trigger = gamepad.left_trigger;
        right_trigger = gamepad.right_trigger;
        circle = gamepad.circle;
        cross = gamepad.cross;
        triangle = gamepad.triangle;
        square = gamepad.square;
        share = gamepad.share;
        options = gamepad.options;
        touchpad = gamepad.touchpad;
        ps = gamepad.ps;
    }
}
