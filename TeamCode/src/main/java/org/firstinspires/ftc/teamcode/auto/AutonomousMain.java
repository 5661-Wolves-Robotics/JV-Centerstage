package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.commands.DetectProp;
import org.firstinspires.ftc.teamcode.auto.commands.DriveToProp;
import org.firstinspires.ftc.teamcode.auto.commands.PushPixel;
import org.firstinspires.ftc.teamcode.bot.CenterStageBot;
import org.firstinspires.ftc.teamcode.bot.subsystems.CenterstageVision;
import org.firstinspires.ftc.teamcode.drive.FieldConstants;
import org.firstinspires.ftc.teamcode.opencv.pipeline.CenterStagePipeline;

@Autonomous
public class AutonomousMain extends LinearOpMode {

    CenterStageBot bot;

    DetectProp detectProp;
    DriveToProp driveToProp;
    PushPixel pushPixel;

    CenterstageVision cv;

    private FieldConstants.Stage stage = FieldConstants.Stage.BACK;
    private FieldConstants.Side side = FieldConstants.Side.RED;

    @Override
    public void runOpMode() {
        bot = new CenterStageBot(hardwareMap);

        cv = new CenterstageVision(hardwareMap, "Camera");

        detectProp = new DetectProp(cv);
        driveToProp = new DriveToProp(bot.drive, cv, () -> side, () -> stage);
        pushPixel = new PushPixel(bot.intake);

        CommandScheduler.getInstance().schedule(
                new RunCommand(telemetry::update),
                new SequentialCommandGroup(
                        detectProp.withTimeout(600),
                        driveToProp,
                        pushPixel.withTimeout(400)
                )
        );

        while(!isStarted()){
            if(gamepad1.dpad_up) stage = FieldConstants.Stage.BACK;
            else if(gamepad1.dpad_down) stage = FieldConstants.Stage.FRONT;

            if(gamepad1.dpad_left) side = FieldConstants.Side.BLUE;
            else if(gamepad1.dpad_right) side = FieldConstants.Side.RED;

            updateTelemetry();
        }

        bot.drive.setPosition(FieldConstants.getFieldStartPose(side, stage));

        while(opModeIsActive() && !isStopRequested()){
            CommandScheduler.getInstance().run();
        }

        CommandScheduler.getInstance().reset();
    }

    public void updateTelemetry(){
        telemetry.addData("Position", stage + " " + side);
        telemetry.update();
    }
}
