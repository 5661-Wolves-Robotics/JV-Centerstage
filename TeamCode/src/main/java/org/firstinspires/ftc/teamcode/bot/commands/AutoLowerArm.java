package org.firstinspires.ftc.teamcode.bot.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.bot.subsystems.ClawArm;
import org.firstinspires.ftc.teamcode.bot.subsystems.DualLinearSlide;

public class AutoLowerArm extends CommandBase {

    private final DualLinearSlide m_slide;
    private final ClawArm m_clawArm;

    public AutoLowerArm(DualLinearSlide slide, ClawArm clawArm){
        m_slide = slide;
        m_clawArm = clawArm;
        addRequirements(clawArm);
    }

    @Override
    public void execute() {
        if(m_slide.getPosition() <= 700 && m_clawArm.getArmState() == ClawArm.ArmState.STORED)
            m_clawArm.setArmState(ClawArm.ArmState.LOWERED);
        else if(m_slide.getPosition() <= 900 && m_clawArm.getArmState() != ClawArm.ArmState.LOWERED){
            m_clawArm.setArmState(ClawArm.ArmState.STORED);
        }
    }
}
