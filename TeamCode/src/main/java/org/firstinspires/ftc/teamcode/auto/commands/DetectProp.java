package org.firstinspires.ftc.teamcode.auto.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.bot.subsystems.CenterstageVision;
import org.firstinspires.ftc.teamcode.opencv.pipeline.CenterStagePipeline;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.TimeUnit;

public class DetectProp extends CommandBase {

    private final CenterstageVision m_cv;
    private final Map<CenterStagePipeline.PropPosition, Integer> frequency = new HashMap<>();

    public DetectProp(CenterstageVision cv){
        m_cv = cv;

        addRequirements(cv);
    }

    @Override
    public void execute() {
        CenterStagePipeline.PropPosition propPos = m_cv.getPipelinePosition();
        Integer num = frequency.get(propPos);
        frequency.put(propPos, num == null ? 1 : num + 1);
    }

    @Override
    public void end(boolean interrupted) {
        m_cv.storePropPosition(Collections.max(frequency.entrySet(), Map.Entry.comparingByValue()).getKey());
    }
}
