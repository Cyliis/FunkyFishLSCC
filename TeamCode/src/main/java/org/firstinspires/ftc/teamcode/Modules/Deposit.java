package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utils.CoolServo;
import org.firstinspires.ftc.teamcode.Utils.IRobotModule;

@Config
public class Deposit implements IRobotModule {
    private CoolServo servo;
    public static String servoName = "deposit";
    public static boolean reversed = false;

    public static double openedPosition = 0;
    public static double closedPosition = 1;

    public static double profileSpeed = 2; // it takes 1/this variable seconds to do the whole range of motion

    public enum State{
        Open(openedPosition), Opening(openedPosition), Closed(closedPosition), Closing(closedPosition);

        double pos;

        State(double pos){
            this.pos = pos;
        }

        static void updatePositions(){
            Open.pos = openedPosition;
            Opening.pos = openedPosition;
            Closed.pos = closedPosition;
            Closing.pos = closedPosition;
        }
    }

    private State state = State.Closed;

    public Deposit(HardwareMap hm){
        servo = new CoolServo(hm, servoName, reversed, profileSpeed, state.pos);
    }

    public void setState(State state){
        if(state == this.state) return;
        this.state = state;
    }

    public State getState(){
        return state;
    }

    private void updateState(){
        switch (this.state){
            case Open:
            case Closed:
                break;
            case Opening:
                if(servo.getTimeToMotionEnd() == 0) setState(State.Open);
                break;
            case Closing:
                if(servo.getTimeToMotionEnd() == 0) setState(State.Closed);
                break;
        }
    }

    private void updateServo(){
        servo.setPosition(state.pos);
        servo.update();
    }

    @Override
    public void update() {
        State.updatePositions();
        updateState();
        updateServo();
    }

    @Override
    public void emergencyStop() {

    }
}
