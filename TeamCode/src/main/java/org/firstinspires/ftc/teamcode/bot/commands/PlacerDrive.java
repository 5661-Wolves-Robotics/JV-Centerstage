package org.firstinspires.ftc.teamcode.bot.commands;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.bot.subsystems.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.Mecanum;

import java.util.function.DoubleSupplier;

public class PlacerDrive extends CommandBase {

    private final PIDFController headingController = new PIDFController(Mecanum.HEADING_PID);

    private final double SPEED = 0.2;
    private final double ANGLE_OFFSET;

    private final Mecanum m_drive;
    private final DoubleSupplier m_horizontal;
    private final DoubleSupplier m_vertical;

    public PlacerDrive(MecanumDriveBase driveBase, DoubleSupplier horizontal, DoubleSupplier vertical, double angleOffset){
        m_drive = driveBase.getDrive();
        m_horizontal = horizontal;
        m_vertical = vertical;
        ANGLE_OFFSET = angleOffset;

        headingController.setInputBounds(-Math.PI, Math.PI);
        headingController.setTargetPosition(Math.PI);

        addRequirements(driveBase);
    }

    @Override
    public void execute() {
        m_drive.setFieldCentricWeightedDrivePower(new Pose2d(
                -m_horizontal.getAsDouble() * SPEED,
                -m_vertical.getAsDouble() * SPEED,
                (headingController.update(m_drive.getPoseEstimate().getHeading()) * DriveConstants.kV) * DriveConstants.TRACK_WIDTH
        ));
    }
}
