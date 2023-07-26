package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CoolServo {

    private final Servo servo;
    private AsymmetricMotionProfile profile;
    private boolean isProfiled = false;

    public CoolServo(HardwareMap hm, String name, boolean reversed, double initialPosition){
        servo = hm.get(Servo.class, name);
        if(reversed) servo.setDirection(Servo.Direction.REVERSE);
        setInitialPosition(initialPosition);
    }

    public CoolServo(HardwareMap hm, String name, boolean reversed, double profileMaxPositiveRate, double profileMaxNegativeRate, double initialPosition){
        servo = hm.get(Servo.class, name);
        if(reversed) servo.setDirection(Servo.Direction.REVERSE);
        profile = new AsymmetricMotionProfile(profileMaxPositiveRate, profileMaxNegativeRate);
        isProfiled = true;
        setInitialPosition(initialPosition);
    }

    public CoolServo(HardwareMap hm, String name, boolean reversed, double profileMaxRate, double initialPosition){
        this(hm, name, reversed, profileMaxRate, profileMaxRate, initialPosition);
    }

    private double cachedPosition, targetPosition;

    private void setInitialPosition(double pos){
        cachedPosition = pos;
        targetPosition = pos;
        servo.setPosition(pos);
    }

    public void setPosition(double position){
        if(position == targetPosition) return;
        targetPosition = position;
        if(isProfiled) profile.setMotion(cachedPosition, targetPosition);
    }

    public void update(){
        if(isProfiled && cachedPosition != profile.getProfilePosition()) {
            cachedPosition = profile.getProfilePosition();
            servo.setPosition(cachedPosition);
        }
        if(!isProfiled && cachedPosition != targetPosition) {
            cachedPosition = targetPosition;
            servo.setPosition(targetPosition);
        }
    }

    public boolean isProfiled() {
        return isProfiled;
    }

    public double getTimeToMotionEnd(){
        if(!isProfiled) return 0;
        return profile.getTimeToMotionEnd();
    }
}
