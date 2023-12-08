package org.firstinspires.ftc.teamcode.auto.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.bot.subsystems.CenterstageVision;
import org.firstinspires.ftc.teamcode.bot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.bot.subsystems.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.Mecanum;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class DriveToProp extends CommandBase {

    private final MecanumDriveBase m_driveBase;
    private final Mecanum m_drive;
    private final Intake m_intake;
    private final CenterstageVision m_cv;

    private final Pose2d LEFT_POS = new Pose2d(26, 2, Math.toRadians(90));
    private final Pose2d CENTER_POS = new Pose2d(30, 4, 0);
    private final Pose2d RIGHT_POS = new Pose2d(26, -2, -Math.toRadians(90));

    public DriveToProp(MecanumDriveBase driveBase, Intake intake, CenterstageVision cv){
        m_driveBase = driveBase;
        m_drive = driveBase.getDrive();
        m_intake = intake;
        m_cv = cv;

        addRequirements(driveBase, intake, cv);
    }

    @Override
    public void initialize() {

        TrajectorySequence sequence = null;

        switch(m_cv.getPropPosition()){
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
