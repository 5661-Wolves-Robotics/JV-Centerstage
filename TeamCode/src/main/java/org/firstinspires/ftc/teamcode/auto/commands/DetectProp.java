package org.firstinspires.ftc.teamcode.auto.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.bot.subsystems.CenterstageVision;
import org.firstinspires.ftc.teamcode.opencv.pipeline.CenterStagePipeline;

public class DetectProp extends CommandBase {

    private final CenterstageVision m_cv;
    private CenterStagePipeline.PropPosition propPosRes;

    public DetectProp(CenterstageVision cv, CenterStagePipeline.PropPosition propPos){
        m_cv = cv;
        propPosRes = propPos;

        addRequirements(cv);
    }

    @Override
    public void initialize() {
        propPosRes = m_cv.getPropPosition();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
