package org.firstinspires.ftc.teamcode.bot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class Delay extends CommandBase {

    private final long m_timeout;
    private Timing.Timer timer;

    public Delay(long timeout){
        m_timeout = timeout;
    }

    @Override
    public void initialize() {
        timer = new Timing.Timer(m_timeout, TimeUnit.MILLISECONDS);
        timer.start();
    }

    @Override
    public boolean isFinished() {
        return timer.done();
    }
}
