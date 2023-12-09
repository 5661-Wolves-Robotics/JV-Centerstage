package org.firstinspires.ftc.teamcode.bot.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.bot.subsystems.DualLinearSlide;

import java.util.function.DoubleSupplier;

public class PowerSlide extends CommandBase {

    private final DualLinearSlide m_slide;
    private final DoubleSupplier m_power;

    public PowerSlide(DualLinearSlide slide, DoubleSupplier power){
        m_slide = slide;
        m_power = power;
        addRequirements(slide);
    }

    @Override
    public void execute() {
        m_slide.setPower(m_power.getAsDouble());
    }
}
