package org.firstinspires.ftc.teamcode.ControlSystems.StateMachines;

public class StateMachine {
    private int nextState = 0;
    private int programState = 0;
    private boolean StageFinished = false;
    public int highestOrdinal = 1000;

    public void start(){
        goToStage(0);
    }

    public void goToStage(int ordinal){

        if(ordinal  > highestOrdinal) {
            ordinal = highestOrdinal;
        }
        nextState = ordinal;
        increment();

    }

    public void setHighestOrdinal(int high){
        highestOrdinal = high;
    }

    public void goToNextStage(){
        goToStage(programState + 1);
    }

    public void increment(){
        programState = nextState;
        StageFinished = true;
    }

    public boolean isStageFinished(){
        if(StageFinished){
            return true;
        }else return false;
    }

    public void setStage(int stage){
        programState = stage;
    }

    public int getStage(){
        return programState;
    }

    public void initializeVariables(){
        if(StageFinished){

            StageFinished = false;

        }else{

        }
    }

}
