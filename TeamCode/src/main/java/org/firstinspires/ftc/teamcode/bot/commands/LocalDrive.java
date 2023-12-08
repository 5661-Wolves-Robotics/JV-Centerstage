package org.firstinspires.ftc.teamcode.bot.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.bot.subsystems.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.Mecanum;

import java.util.function.DoubleSupplier;

public class LocalDrive extends CommandBase {

    private final Mecanum m_drive;
    private final DoubleSupplier m_x;
    private final DoubleSupplier m_y;
    private final DoubleSupplier m_rot;

    public LocalDrive(MecanumDriveBase driveBase, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot){
        m_drive = driveBase.getDrive();
        m_x = x;
        m_y = y;
        m_rot = rot;

        addRequirements(driveBase);
    }

    @Override
    public void execute() {
        m_drive.setWeightedDrivePower(new Pose2d(
                m_y.getAsDouble(),
                -m_x.getAsDouble(),
                -m_rot.getAsDouble()
        ));
    }
}
