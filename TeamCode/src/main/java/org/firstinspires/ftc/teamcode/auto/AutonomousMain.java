package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.commands.DetectProp;
import org.firstinspires.ftc.teamcode.auto.commands.DriveToProp;
import org.firstinspires.ftc.teamcode.auto.commands.PushPixel;
import org.firstinspires.ftc.teamcode.bot.CenterStageBot;
import org.firstinspires.ftc.teamcode.opencv.pipeline.CenterStagePipeline;

@Autonomous
public class AutonomousMain extends CommandOpMode {

    CenterStageBot bot;

    DetectProp detectProp;
    DriveToProp driveToProp;
    PushPixel pushPixel;

    CenterStagePipeline.PropPosition propPos = null;

    @Override
    public void initialize() {
        bot = new CenterStageBot(hardwareMap);

        telemetry.addData("PropPosition", bot.cv::getPropPosition);
        schedule(new RunCommand(telemetry::update));

        detectProp = new DetectProp(bot.cv, propPos);
        driveToProp = new DriveToProp(bot.drive, this::getPropPos);
        pushPixel = new PushPixel(bot.drive, bot.intake);

        schedule(
                new SequentialCommandGroup(
                        detectProp,
                        driveToProp,
                        pushPixel
                )
        );

        telemetry.addLine("READY");
        telemetry.update();
    }

    private CenterStagePipeline.PropPosition getPropPos(){
        return propPos;
    }
}
