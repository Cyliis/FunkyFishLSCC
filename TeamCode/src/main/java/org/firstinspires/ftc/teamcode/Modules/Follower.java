package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Utils.IRobotModule;
import org.firstinspires.ftc.teamcode.Utils.Trajectory;

@Config
public class Follower implements IRobotModule {

    private Trajectory trajectory;
    private final MecanumDrive mecanumDrive;
    private final Localizer localizer;

    public Follower(MecanumDrive mecanumDrive, Localizer localizer){
        this.mecanumDrive = mecanumDrive;
        this.localizer = localizer;
        this.trajectory = new Trajectory(localizer.getPose());

        mecanumDrive.setRunMode(MecanumDrive.RunMode.PID);
    }

    public void setTrajectory(Trajectory trajectory){
        this.trajectory = trajectory;
    }

    @Override
    public void update() {
        if(trajectory.getCurrentPoint().isReached(localizer.getPose())){
            mecanumDrive.setTargetPose(trajectory.getNextPoint());
        }
    }

    @Override
    public void emergencyStop() {

    }
}
