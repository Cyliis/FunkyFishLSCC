package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utils.CoolDigitalSensor;
import org.firstinspires.ftc.teamcode.Utils.IRobotModule;

public class HorizontalExtension implements IRobotModule {

    public static double inPosition = 0, scorePosition = 200;
    public static double maxExtension = 400;

    public ElapsedTime timeSinceLastStateChange;

    public static double resetStep = 10;

    public static String limitSwitchName = "hExtLimitSwitch";

    private final CoolDigitalSensor limitSwitch;

    private static double offset = 0;

    private final DiffyCrane crane;

    public enum State{
        In(inPosition), GoingIn(inPosition), GoingScore(scorePosition), Score(scorePosition),
        GoingToReset(inPosition), Reset(inPosition);
        double pos;
        State(double pos){
            this.pos = pos;
        }
        double getPos(){
            return pos;
        }
        double getOffsetPos(){
            return pos + offset;
        }
        void setPos(double pos){
            this.pos = pos;
        }
    }

    public HorizontalExtension(HardwareMap hm, DiffyCrane crane){
        limitSwitch = new CoolDigitalSensor(hm, limitSwitchName);
        this.crane = crane;
    }

    private State currentState = State.GoingIn;

    public void setState(State state){
        if(currentState == state) return;
        this.currentState = state;
        timeSinceLastStateChange.reset();
    }

    public State getState(){
        return currentState;
    }

    @Override
    public void updateState() {
        switch (currentState){
            case In:
            case Score:
                break;
            case Reset:
                if(limitSwitch.getState()) setState(State.In);
                else {
                    offset -= resetStep;
                    setState(State.GoingToReset);
                }
                break;
            case GoingToReset:
                if(Math.abs(crane.getCurrentHorizontalPosition() - State.Reset.getOffsetPos()) <= DiffyCrane.positionTolerance)
                    setState(State.Reset);
                break;
            case GoingIn:
                if(Math.abs(crane.getCurrentHorizontalPosition() - State.In.getOffsetPos()) <= DiffyCrane.positionTolerance)
                    setState(State.Reset);
                break;
            case GoingScore:
                if(Math.abs(crane.getCurrentHorizontalPosition() - State.Score.getOffsetPos()) <= DiffyCrane.positionTolerance)
                    setState(State.Score);
                break;
        }
    }

    @Override
    public void update() {
        updateState();
        crane.setTargetHorizontalPosition(currentState.getOffsetPos());
    }

    @Override
    public void emergencyStop() {

    }
}
