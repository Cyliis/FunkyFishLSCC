package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utils.CoolDigitalSensor;
import org.firstinspires.ftc.teamcode.Utils.IRobotModule;

public class Intake implements IRobotModule {
    private Deposit deposit;
    private CoolDigitalSensor depositSensor;
    private IntakeWheels intakeWheels;
    private Ramp ramp;

    public Intake(HardwareMap hm, Deposit deposit, CoolDigitalSensor depositSensor){
        this.intakeWheels = new IntakeWheels(hm);
        this.ramp = new Ramp(hm);
        this.deposit = deposit;
        this.depositSensor = depositSensor;
    }

    public enum State{
        Idle, StartIntake, RampDownDepositOpen, Intaking, CloseDeposit
    }

    private State state = State.Idle;

    public void setState(State state){
        if(state == this.state) return;
        switch (state){
            case Idle:
                ramp.setState(Ramp.State.GoingUp);
                intakeWheels.setState(IntakeWheels.State.Off);
                break;
        }
        this.state = state;
    }

    public State getState(){
        return state;
    }

    private void updateState(){
        switch (this.state){
            case StartIntake:
                if(ramp.getState()!= Ramp.State.Down) ramp.setState(Ramp.State.GoingDown);
                if(deposit.getState()!= Deposit.State.Open) deposit.setState(Deposit.State.Opening);
                setState(State.RampDownDepositOpen);
                break;
            case RampDownDepositOpen:
                if(ramp.getState() == Ramp.State.Down && deposit.getState()== Deposit.State.Open)
                    intakeWheels.setState(IntakeWheels.State.Running);
                setState(State.Intaking);
                break;
            case Intaking:
                if(!depositSensor.getState()) deposit.setState(Deposit.State.Closing);
                break;
            case CloseDeposit:
                if(deposit.getState()== Deposit.State.Closed)
                    setState(State.Idle);
                break;
        }
    }

    @Override
    public void update() {

    }

    @Override
    public void emergencyStop() {

    }
}
