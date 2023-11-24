package org.firstinspires.ftc.teamcode.bot.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.bot.subsystems.DualLinearSlide;

public class MoveSlideToPosition extends CommandBase {

    private final int m_pos;
    private final DualLinearSlide m_slide;

    public MoveSlideToPosition(DualLinearSlide slide, int pos){
        m_pos = pos;
        m_slide = slide;
        addRequirements(slide);
    }

    @Override
    public void initialize() {
        m_slide.setTargetPosition(m_pos);
    }

    @Override
    public boolean isFinished() {
        return !m_slide.isBusy();
    }
}
