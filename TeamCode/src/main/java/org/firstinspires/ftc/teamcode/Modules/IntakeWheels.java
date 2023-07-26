package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utils.CoolMotor;
import org.firstinspires.ftc.teamcode.Utils.IRobotModule;

@Config
public class IntakeWheels implements IRobotModule {
    private CoolMotor motor;
    public static String motorName = "intake";
    public static boolean reversed = false;

    public static double intakePower = 1;

    public enum State{
        Running(intakePower), Off(0);

        double power;

        State(double power){
            this.power = power;
        }

        static void updatePowers(){
            Running.power = intakePower;
        }
    }

    private State state = State.Off;

    public IntakeWheels(HardwareMap hm){
        motor = new CoolMotor(hm, motorName, CoolMotor.RunMode.RUN, reversed);
    }

    public void setState(State state){
        if(this.state == state) return;
        this.state = state;
    }

    public State getState(){
        return state;
    }

    private void updateMotor(){
        motor.setPower(state.power);
        motor.update();
    }

    @Override
    public void update() {
        State.updatePowers();
        updateMotor();
    }

    @Override
    public void emergencyStop() {

    }
}
