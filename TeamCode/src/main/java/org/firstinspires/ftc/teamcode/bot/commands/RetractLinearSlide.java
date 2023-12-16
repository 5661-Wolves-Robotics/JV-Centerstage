package org.firstinspires.ftc.teamcode.bot.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.bot.subsystems.ClawArm;
import org.firstinspires.ftc.teamcode.bot.subsystems.DualLinearSlide;

public class RetractLinearSlide extends SequentialCommandGroup {

    public RetractLinearSlide(DualLinearSlide slide, ClawArm clawArm){
        addCommands(
                new MoveArm(clawArm, ClawArm.ArmState.RAISED),
                new WaitCommand(200),
                new MoveSlideToPosition(slide, 0)
        );
        addRequirements(slide, clawArm);
    }
}
