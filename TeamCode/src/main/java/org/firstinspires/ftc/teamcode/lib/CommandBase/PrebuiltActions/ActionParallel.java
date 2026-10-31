package org.firstinspires.ftc.teamcode.lib.CommandBase.PrebuiltActions;

import org.firstinspires.ftc.teamcode.lib.CommandBase.Action;

public class ActionParallel implements Action {
    public enum TERMINATIONTYPE {
        ANY,
        ALL,
        FIRST,
        NONE
    }

    TERMINATIONTYPE terminationType;
    Action[] actionList;

    public ActionParallel(TERMINATIONTYPE terminationType, Action... actions) {
        this.terminationType = terminationType;
        actionList = actions;
    }

    public Action[] getActionList() {
        return actionList;
    }

    public void setActionList(Action... actions) {
        actionList = actions;
    }

    //Implementations
    @Override
    public void init() {
        for (Action action : actionList) {
            action.init();
        }
    }

    @Override
    public void loop() {
        for (Action action : actionList) {
            action.loop();
        }
    }

    @Override
    public void shutdown() {
        for (Action action : actionList) {
            action.shutdown();
        }
    }

    @Override
    public boolean finished() {
        switch (terminationType) {
            case ANY:
                for (Action action : actionList) {
                    if (action.finished()) {
                        return true; //If any single action is finished, whole action is finished
                    }
                }
                return false;

            case ALL:
                for (Action action : actionList) {
                    if (!action.finished()) {
                        return false; //If any single action is not finished, whole action must not be finished
                    }
                }
                return true;

            case FIRST:
                return actionList[0].finished();

            default:
                return false;
        }
    }
}
