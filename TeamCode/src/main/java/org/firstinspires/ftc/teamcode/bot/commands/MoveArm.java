package org.firstinspires.ftc.teamcode.bot.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.bot.subsystems.ClawArm;

public class MoveArm extends CommandBase {

    private final ClawArm m_clawArm;
    private final ClawArm.ArmState m_armState;

    public MoveArm(ClawArm clawArm, ClawArm.ArmState armState){
        m_clawArm = clawArm;
        m_armState = armState;
        addRequirements(clawArm);
    }

    @Override
    public void initialize() {
        m_clawArm.setArmState(m_armState);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
