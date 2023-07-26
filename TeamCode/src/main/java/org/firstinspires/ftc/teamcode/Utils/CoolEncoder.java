package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CoolEncoder {
    private final DcMotorEx motor;
    private boolean reversed = false;

    private static int offset;

    public CoolEncoder(HardwareMap hm, String hmName, boolean reversed){
        this.motor = hm.get(DcMotorEx.class, hmName);
        this.reversed = reversed;
    }

    public CoolEncoder(HardwareMap hm, String hmName){
        this.motor = hm.get(DcMotorEx.class, hmName);
        this.reversed = false;
    }

    public int getCurrentPosition(){
        if(reversed && motor.getDirection() == DcMotorSimple.Direction.FORWARD) return -(motor.getCurrentPosition() - offset);
        return motor.getCurrentPosition() - offset;
    }

    public int getRawCurrentPosition(){
        if(reversed && motor.getDirection() == DcMotorSimple.Direction.FORWARD) return -motor.getCurrentPosition();
        return motor.getCurrentPosition();
    }

    public double getVelocity(){
        if(reversed) return -motor.getVelocity();
        else return motor.getVelocity();
    }

    public void reset(){
        offset = motor.getCurrentPosition();
    }

    public void setReversed(boolean rev){
        this.reversed = rev;
    }
}
