package org.firstinspires.ftc.teamcode.auto.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.bot.subsystems.CenterstageVision;
import org.firstinspires.ftc.teamcode.bot.subsystems.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.Mecanum;
import org.firstinspires.ftc.teamcode.opencv.pipeline.PropPositionSupplier;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class DriveToProp extends CommandBase {

    private final Mecanum m_drive;
    private final CenterstageVision m_cv;

    public DriveToProp(MecanumDriveBase driveBase, CenterstageVision cv){
        m_drive = driveBase.getDrive();
        m_cv = cv;

        addRequirements(driveBase, cv);
    }

    @Override
    public void initialize() {
        switch(m_cv.getPropPosition()){
            case LEFT:
                m_drive.followTrajectory(m_drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                        .splineTo(new Vector2d(26, 2), Math.toRadians(90))
                        .build());
                break;
            case CENTER:
                m_drive.followTrajectory(m_drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                        .splineTo(new Vector2d(30, 2), 0)
                        .build());
                break;
            case RIGHT:
                m_drive.followTrajectory(m_drive.trajectoryBuilder(new Pose2d(0, 0, 0), -Math.toRadians(90))
                        .splineTo(new Vector2d(24, -24), 0)
                        .build());
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return !m_drive.isBusy();
    }
}
