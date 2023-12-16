package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.commands.DetectProp;
import org.firstinspires.ftc.teamcode.auto.commands.DriveToProp;
import org.firstinspires.ftc.teamcode.auto.commands.PushPixel;
import org.firstinspires.ftc.teamcode.bot.CenterStageBot;
import org.firstinspires.ftc.teamcode.bot.subsystems.CenterstageVision;
import org.firstinspires.ftc.teamcode.opencv.pipeline.CenterStagePipeline;

@Autonomous
public class AutonomousMain extends CommandOpMode {

    CenterStageBot bot;

    DetectProp detectProp;
    DriveToProp driveToProp;
    PushPixel pushPixel;
    CenterstageVision cv;

    @Override
    public void initialize() {
        bot = new CenterStageBot(hardwareMap);

        cv = new CenterstageVision(hardwareMap, "Camera");

        detectProp = new DetectProp(cv);
        driveToProp = new DriveToProp(bot.drive, cv);
        pushPixel = new PushPixel(bot.intake);

        schedule(
                new SequentialCommandGroup(
                        detectProp.withTimeout(600),
                        driveToProp,
                        pushPixel.withTimeout(400)
                )
        );

        telemetry.addData("PropPosition", cv::getPropPosition);
        schedule(new RunCommand(telemetry::update));
    }
}
