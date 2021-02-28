package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.Queue;

public class Actions {
    protected Queue<Runnable> actions = new LinkedList<>();
    private Telemetry telemetry;
    private volatile boolean shouldRun;
    protected final Object lock = new Object();

    public Actions() {
        this.shouldRun = true;
    }

    public Object add(Runnable action) {
        synchronized (lock) {
            this.actions.add(action);
            return action;
        }
    }

    public boolean remove(Object token) {
        synchronized (lock) {
            return this.actions.remove((Runnable)token);
        }
    }

    public void run() {
        synchronized (lock) {
            //telemetry.addData("Actions", "Running actions on thread: " + Thread.currentThread().getId());
            Iterator<Runnable> iterator = actions.iterator();
            while (iterator.hasNext() && shouldRun) {
                Runnable action = actions.poll();
                if (action != null) {
                    action.run();
                }
            }
        }
    }

    public void stop() {
        shouldRun = false;
    }
}