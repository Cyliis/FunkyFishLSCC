package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CoolServo {

    private final Servo servo;
    private AsymmetricMotionProfile profile;
    private boolean isProfiled = false;

    public CoolServo(HardwareMap hm, String name){
        servo = hm.get(Servo.class, name);
    }

    public CoolServo(HardwareMap hm, String name, double profileMaxPositiveRate, double profileMaxNegativeRate){
        servo = hm.get(Servo.class, name);
        profile = new AsymmetricMotionProfile(profileMaxPositiveRate, profileMaxNegativeRate);
        isProfiled = true;
    }

    public CoolServo(HardwareMap hm, String name, double profileMaxRate){
        this(hm, name, profileMaxRate, profileMaxRate);
    }

    public void setDirection(Servo.Direction direction){
        servo.setDirection(direction);
    }

    private double cachedPosition, targetPosition;

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

}
