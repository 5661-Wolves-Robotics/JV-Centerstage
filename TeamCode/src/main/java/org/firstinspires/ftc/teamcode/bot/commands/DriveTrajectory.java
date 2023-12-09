package org.firstinspires.ftc.teamcode.bot.commands;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.bot.subsystems.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.Mecanum;

public class DriveTrajectory extends CommandBase {

    private final MecanumDriveBase m_driveBase;
    private final Mecanum m_drive;
    private final Trajectory m_trajectory;

    public DriveTrajectory(MecanumDriveBase driveBase, Trajectory trajectory){
        m_driveBase = driveBase;
        m_drive = driveBase.getDrive();
        m_trajectory = trajectory;

        addRequirements(driveBase);
    }

    @Override
    public void initialize() {
        m_drive.followTrajectory(m_trajectory);
    }

    @Override
    public boolean isFinished() {
        return !m_drive.isBusy();
    }
}
