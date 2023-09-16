package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utils.IRobotModule;
import org.firstinspires.ftc.teamcode.Utils.Pose;
import org.firstinspires.ftc.teamcode.Wrappers.CoolIMU;
import org.firstinspires.ftc.teamcode.drive.FunnyLocalizer;

@Config
public class Localizer implements IRobotModule {
    protected Pose pose;
    private FunnyLocalizer localizer;
    public CoolIMU imu;

    public Localizer(HardwareMap hm, Pose initialPose){
        this.pose = initialPose;
        this.imu = new CoolIMU(hm);
        this.localizer = new FunnyLocalizer(hm, imu);
        localizer.setPoseEstimate(new Pose2d(initialPose.getX(), initialPose.getY(), initialPose.getHeading()));
    }

    public void setPose(Pose pose){
        this.pose = pose;
    }

    public Pose getPoseEstimate(){
        return pose;
    }

    @Override
    public void update() {
        localizer.update();
        Pose2d pose2d = localizer.getPoseEstimate();
        pose = new Pose(pose2d.getX(), pose2d.getY(), pose2d.getHeading());
    }

    @Override
    public void emergencyStop() {

    }
}
