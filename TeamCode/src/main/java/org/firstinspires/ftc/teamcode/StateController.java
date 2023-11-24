package org.firstinspires.ftc.teamcode;

import java.util.HashMap;
import java.util.Map;

public class StateController<T> {

    Map<T, StateFunc> stateFuncs = new HashMap<>();

    public interface StateFunc{
        void run();
    }

    T currState;

    public StateController(T state){
        currState = state;
    }

    public void addState(T state, StateFunc func){
        stateFuncs.put(state, func);
    }

    public void update(){
        stateFuncs.get(currState).run();
    }

    public void setState(T newState){
        currState = newState;
    }

    public T getState(){
        return currState;
    }
}
