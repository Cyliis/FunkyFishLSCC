package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.CoolServo;
import org.firstinspires.ftc.teamcode.Utils.IRobotModule;

@Config
public class HorizontalExtension implements IRobotModule {
    private CoolServo servo1, servo2;
    public static String servo1Name = "servo1", servo2Name = "servo2";
    public static boolean servo1Reversed = false, servo2Reversed = true;

    public static double inPosition = 0;
    public static double maxOutPosition = 0.8;
    public static double intermediaryOutPosition = 0.5;
    public static double currentOutPosition = intermediaryOutPosition;

    public static double maxSpeed = 2;

    public enum State{
        In(inPosition), GoingIn(inPosition), Out(currentOutPosition), GoingOut(currentOutPosition);

        double pos;

        State(double pos){
            this.pos = pos;
        }

        static void updatePositions(){
            In.pos = inPosition;
            GoingIn.pos = inPosition;
            Out.pos = currentOutPosition;
            GoingOut.pos = currentOutPosition;
        }
    }

    private State state = State.In;

    public HorizontalExtension(HardwareMap hm){
        servo1 = new CoolServo(hm, servo1Name, servo1Reversed, maxSpeed, state.pos);
        servo2 = new CoolServo(hm, servo2Name, servo2Reversed, maxSpeed, state.pos);
    }

    private void setState(State state){
        if(state == this.state) return;
        this.state = state;
    }

    public State getState(){
        return state;
    }

    private void updateState(){
        switch (this.state){
            case In:
            case Out:
                break;
            case GoingIn:
                if(servo1.getTimeToMotionEnd() == 0 && servo2.getTimeToMotionEnd() == 0) setState(State.In);
                break;
            case GoingOut:
                if(servo1.getTimeToMotionEnd() == 0 && servo2.getTimeToMotionEnd() == 0) setState(State.Out);
                break;
        }
    }

    private void updateServos(){
        servo1.setPosition(state.pos);
        servo2.setPosition(state.pos);
        servo1.update();
        servo2.update();
    }

    @Override
    public void update() {
        State.updatePositions();
        updateState();
        updateServos();
    }

    @Override
    public void emergencyStop() {

    }
}
