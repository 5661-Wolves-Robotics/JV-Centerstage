package org.firstinspires.ftc.teamcode.auto.commands;

import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.bot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.bot.subsystems.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.Mecanum;

public class PushPixel extends SequentialCommandGroup {

    private final Mecanum m_drive;

    public PushPixel(MecanumDriveBase driveBase, Intake intake){
        m_drive = driveBase.getDrive();

        addCommands(
                new InstantCommand(intake::reverse, intake).withTimeout(400),
                new InstantCommand(intake::stop, intake),
                new FunctionalCommand(
                        () -> m_drive.followTrajectory(m_drive.trajectoryBuilder(m_drive.getPoseEstimate())
                                .back(2)
                                .build()
                        )
                        ,
                        null,
                        null,
                        m_drive::done,
                        driveBase
                )
        );

        addRequirements(driveBase, intake);
    }

}
