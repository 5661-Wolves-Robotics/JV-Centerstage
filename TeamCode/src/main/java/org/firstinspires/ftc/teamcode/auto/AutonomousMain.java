package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.commands.DetectProp;
import org.firstinspires.ftc.teamcode.bot.CenterStageBot;
import org.firstinspires.ftc.teamcode.bot.subsystems.CenterstageVision;
import org.firstinspires.ftc.teamcode.drive.FieldConstants;

@Autonomous
public class AutonomousMain extends CommandOpMode {

    CenterStageBot bot;

    DetectProp detectProp;

    CenterstageVision cv;

    FieldConstants.Stage stage = FieldConstants.Stage.BACK;
    FieldConstants.Side side = FieldConstants.Side.RED;

    @Override
    public void initialize() {
        bot = new CenterStageBot(hardwareMap);

        cv = new CenterstageVision(hardwareMap, "Camera");

        detectProp = new DetectProp(cv);

        schedule(
                new RunCommand(telemetry::update),
                new SequentialCommandGroup(
                        detectProp.withTimeout(1000)
                )
        );

        while(!isStarted() && !isStopRequested()){
            if(gamepad1.dpad_up) stage = FieldConstants.Stage.BACK;
            else if(gamepad1.dpad_down) stage = FieldConstants.Stage.FRONT;

            if(gamepad1.dpad_left) side = FieldConstants.Side.BLUE;
            else if(gamepad1.dpad_right) side = FieldConstants.Side.RED;
            updateTelemetry();

            if(gamepad1.a) break;
        }

        cv.extractColorChannel(side == FieldConstants.Side.RED ? 1 : 2);
        bot.drive.setPosition(FieldConstants.getFieldStartPose(side, stage));
    }

    public void updateTelemetry(){
        telemetry.addData("Position", stage + " " + side);
        telemetry.update();
    }
}
