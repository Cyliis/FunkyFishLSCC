package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utils.CoolServo;
import org.firstinspires.ftc.teamcode.Utils.IRobotModule;

@Config
public class Ramp implements IRobotModule {
    private CoolServo servo;
    public static String servoName = "ramp";
    public static boolean reversed = false;

    public static double upPosition = 0;
    public static double downPosition = 1;

    public static double profileSpeed = 2; // it takes 1/this variable seconds to do the whole range of motion

    public enum State{
        Up(upPosition), GoingUp(upPosition), Down(downPosition), GoingDown(downPosition);

        double pos;

        State(double pos){
            this.pos = pos;
        }

        static void updatePositions(){
            Up.pos = upPosition;
            GoingUp.pos = downPosition;
            Down.pos = downPosition;
            GoingDown.pos = downPosition;
        }
    }

    private State state = State.Up;

    public Ramp(HardwareMap hm){
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
            case Up:
            case Down:
                break;
            case GoingUp:
                if(servo.getTimeToMotionEnd() == 0) setState(State.Up);
                break;
            case GoingDown:
                if(servo.getTimeToMotionEnd() == 0) setState(State.Down);
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
