package org.firstinspires.ftc.teamcode.Modules;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utils.CoolEncoder;
import org.firstinspires.ftc.teamcode.Utils.CoolIMU;
import org.firstinspires.ftc.teamcode.Utils.IRobotModule;
import org.firstinspires.ftc.teamcode.Utils.Pose;
import org.firstinspires.ftc.teamcode.Utils.Vector2d;

@Config
public class Localizer implements IRobotModule {

    public CoolIMU imu;
    private CoolEncoder odo;
    public static boolean reversedOdoEncoder = false;

    //all distance units in meters (I hate freedom/j)
    private final double wheelDiameter = 0.035, lateralOffset = 0, ticksPerRev = 8192;
    private final double metersPerTick = wheelDiameter*2*PI / ticksPerRev, ticksPerRad = lateralOffset / metersPerTick;

    Pose pose;

    public static String imuName = "imu", odoEncoderName = "odo";

    public Localizer(HardwareMap hm){
        this.imu = new CoolIMU(hm);
        this.odo = new CoolEncoder(hm, odoEncoderName, reversedOdoEncoder);
    }

    public void setPose(Pose pose){
        this.pose = pose;
    }

    public Pose getPose(){
        return pose;
    }

    private void updatePose(){
        double currentHeading = imu.getHeading();
        double deltaHeading = currentHeading - pose.getHeading();
        double arcLength = (odo.getCurrentPosition() - ticksPerRad * deltaHeading) * metersPerTick;
        double arcRadius = arcLength/deltaHeading;

        double deltaX = (Math.cos(currentHeading) - Math.cos(pose.getHeading())) * arcRadius;
        double deltaY = (Math.sin(currentHeading) - Math.sin(pose.getHeading())) * arcRadius;

        Vector2d newPosition = new Vector2d(pose.getPosition().getX() + deltaX, pose.getPosition().getY() + deltaY);

        pose.setPosition(newPosition);
        pose.setHeading(currentHeading);
    }

    @Override
    public void update() {
        updatePose();
    }

    @Override
    public void emergencyStop() {

    }
}
