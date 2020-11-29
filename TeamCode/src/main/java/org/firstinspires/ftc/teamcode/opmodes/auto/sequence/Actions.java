package org.firstinspires.ftc.teamcode.opmodes.auto.sequence;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.auto.Constants;
import org.firstinspires.ftc.teamcode.opmodes.auto.FullAuto;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.Queue;

public class Actions {
    protected Queue<Runnable> actions = new LinkedList<>();
    private Telemetry telemetry;
    private volatile boolean shouldRun;
    protected final Object theLock = new Object();

    public Actions(Telemetry tel) {
        this.telemetry = tel;
        this.shouldRun = true;
    }

    public Object add(Runnable action) {
        synchronized (theLock) {
            this.actions.add(action);
            return action;
        }
    }

    public boolean remove(Object token) {
        synchronized (theLock) {
            return this.actions.remove((Runnable)token);
        }
    }

    public void run(String inputAction) {
        synchronized (theLock) {
            //telemetry.addData("Actions", "Running actions on thread: " + Thread.currentThread().getId());
            if (!inputAction.equals(Constants.AllTrajectories)) {
                isolateAction(inputAction);
            }

            Iterator<Runnable> iterator = actions.iterator();
            while (iterator.hasNext() && shouldRun) {
                Runnable action = actions.poll();
                if (action != null) {
                    action.run();
                }
            }

        }
    }

    private void isolateAction(String inputAction){
        int foundActions = 0;
        Iterator<Runnable> iterator = actions.iterator();
        while (iterator.hasNext() && shouldRun) {
            Runnable action = actions.poll();
            if (action != null) {
                if (!action.toString().equalsIgnoreCase(inputAction)){
                    actions.remove(action);
                } else foundActions++;
            }
        }
        if (foundActions == 0) telemetry.addLine(FullAuto.TrajectorySelect + " sequence not found");
    }

    public void stop() {
        shouldRun = false;
    }
}