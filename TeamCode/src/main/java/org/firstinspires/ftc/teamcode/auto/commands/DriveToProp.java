package org.firstinspires.ftc.teamcode.auto.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.bot.subsystems.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.Mecanum;
import org.firstinspires.ftc.teamcode.opencv.pipeline.PropPositionSupplier;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class DriveToProp extends CommandBase {

    private final Mecanum m_drive;
    private final PropPositionSupplier m_propPositionSupplier;

    public DriveToProp(MecanumDriveBase driveBase, PropPositionSupplier propPositionSupplier){
        m_drive = driveBase.getDrive();
        m_propPositionSupplier = propPositionSupplier;

        addRequirements(driveBase);
    }

    @Override
    public void initialize() {

        TrajectorySequence sequence = null;

        switch(m_propPositionSupplier.getEnum()){
            case LEFT:
                sequence = m_drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        .lineTo(new Vector2d(26, 0))
                        .turn(Math.toRadians(90))
                        .forward(2)
                        .build();
                break;
            case CENTER:
                sequence = m_drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        .splineTo(new Vector2d(30, 2), 0)
                        .build();
                break;
            case RIGHT:
                sequence = m_drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        .lineTo(new Vector2d(26, 0))
                        .turn(-Math.toRadians(90))
                        .forward(2)
                        .build();
                break;
        }

        m_drive.followTrajectorySequence(sequence);
    }

    @Override
    public boolean isFinished() {
        return !m_drive.isBusy();
    }
}
