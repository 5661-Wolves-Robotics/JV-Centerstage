package org.firstinspires.ftc.teamcode.auto.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.bot.subsystems.CenterstageVision;
import org.firstinspires.ftc.teamcode.bot.subsystems.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.FieldConstants;
import org.firstinspires.ftc.teamcode.drive.Mecanum;
import org.firstinspires.ftc.teamcode.opencv.pipeline.CenterStagePipeline;

public class DriveToProp extends CommandBase {

    private final Mecanum m_drive;
    private final CenterstageVision m_cv;

    private final FieldConstants.EnumSupplier<FieldConstants.Side> m_sideSupplier;
    private final FieldConstants.EnumSupplier<FieldConstants.Stage> m_stageSupplier;

    public DriveToProp(
            MecanumDriveBase driveBase,
            CenterstageVision cv,
            FieldConstants.EnumSupplier<FieldConstants.Side> sideSupplier,
            FieldConstants.EnumSupplier<FieldConstants.Stage> stageSupplier
    ){
        m_drive = driveBase.getDrive();
        m_cv = cv;

        m_sideSupplier = sideSupplier;
        m_stageSupplier = stageSupplier;

        addRequirements(driveBase, cv);
    }

    @Override
    public void initialize() {
        CenterStagePipeline.PropPosition propPos = m_cv.getPropPosition();

        FieldConstants.Side side = m_sideSupplier.getVal();
        boolean redSide = side == FieldConstants.Side.RED;
        double xScale = redSide ? 1 : -1;

        //For BLUE side swap left and right prop positions
        if(!redSide){
            if(propPos == CenterStagePipeline.PropPosition.LEFT) propPos = CenterStagePipeline.PropPosition.RIGHT;
            else if(propPos == CenterStagePipeline.PropPosition.RIGHT) propPos = CenterStagePipeline.PropPosition.LEFT;
        }

        switch(propPos){
            case LEFT:
                m_drive.followTrajectory(m_drive.trajectoryBuilder(m_drive.getPoseEstimate())
                        .splineTo(new Vector2d(8, -34 * xScale), Math.toRadians(180))
                        .build());
                break;
            case CENTER:
                m_drive.followTrajectory(m_drive.trajectoryBuilder(m_drive.getPoseEstimate())
                        .splineTo(new Vector2d(4, -30 * xScale), Math.toRadians(90 * xScale))
                        .build());
                break;
            case RIGHT:
                m_drive.followTrajectory(m_drive.trajectoryBuilder(m_drive.getPoseEstimate(), Math.toRadians(360))
                        .splineTo(new Vector2d(31, -34 * xScale), Math.toRadians(90 * xScale))
                        .build());
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return !m_drive.isBusy();
    }
}
