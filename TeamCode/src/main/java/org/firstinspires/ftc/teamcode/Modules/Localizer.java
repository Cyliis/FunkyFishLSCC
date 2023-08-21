package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utils.IRobotModule;
import org.firstinspires.ftc.teamcode.Utils.Pose;

@Config
public class Localizer implements IRobotModule {
    private Pose pose;

    public Localizer(HardwareMap hm, Pose initialPose){
        this.pose = initialPose;
    }

    public void setPose(Pose pose){
        this.pose = pose;
    }

    public Pose getPose(){
        return pose;
    }

    @Override
    public void update() {
        pose = new Pose(0,0,0);
    }

    @Override
    public void emergencyStop() {

    }
}
