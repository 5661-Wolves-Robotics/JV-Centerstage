package org.firstinspires.ftc.teamcode.bot.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.bot.subsystems.Intake;

public class ToggleIntake extends CommandBase {

    private final Intake m_intake;

    public ToggleIntake(Intake intake){
        m_intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if(m_intake.getState() == Intake.IntakeState.LOWERED){
            m_intake.setState(Intake.IntakeState.RAISED);
            m_intake.disable();
        } else {
            m_intake.setState(Intake.IntakeState.LOWERED);
            m_intake.power();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
