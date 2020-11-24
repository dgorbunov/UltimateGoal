package org.firstinspires.ftc.teamcode.opmodes.auto.sequence;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.Queue;

public class Actions {
    protected Queue<Runnable> actions = new LinkedList<Runnable>();
    Telemetry telemetry;
    private boolean shouldRun;
    protected final Object theLock = new Object();
    public final int MaxTimeoutInMsec = 300000;

    public Actions(Telemetry tel) {
        this.telemetry  = tel;
        this.shouldRun = true;
    }

    public Object addAction(Runnable action) {
        synchronized (theLock) {
            this.actions.add(action);
            return action;
        }
    }

    public boolean removeAction(Object token) {
        synchronized (theLock) {
            return this.actions.remove((Runnable)token);
        }
    }

    public void run() {
        synchronized (theLock) {
            Iterator iterator = actions.iterator();
            while (iterator.hasNext() && shouldRun) {
                Runnable action = actions.poll();
                action.run();
            }
        }
    }

    public void stop() {
        shouldRun = false;
    }
}