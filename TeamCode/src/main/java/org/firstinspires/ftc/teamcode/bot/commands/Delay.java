package org.firstinspires.ftc.teamcode.bot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Delay extends CommandBase {

    private final double m_timeout;
    private ElapsedTime time = null;

    public Delay(double timeout){
        m_timeout = timeout;
    }

    @Override
    public void initialize() {
        time = new ElapsedTime();
    }

    @Override
    public boolean isFinished() {
        return time.seconds() >= m_timeout;
    }
}
