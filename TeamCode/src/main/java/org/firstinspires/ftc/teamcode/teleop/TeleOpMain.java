package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bot.CenterStageBot;
import org.firstinspires.ftc.teamcode.bot.commands.LocalDrive;
import org.firstinspires.ftc.teamcode.bot.commands.PlacerDrive;

@TeleOp
public class TeleOpMain extends CommandOpMode {

    GamepadEx driver, placer;
    LocalDrive localDrive;
    PlacerDrive placerDrive;

    @Override
    public void initialize() {
        CenterStageBot bot = new CenterStageBot(hardwareMap);

        driver = new GamepadEx(gamepad1);
        placer = new GamepadEx(gamepad2);

        localDrive = new LocalDrive(bot.drive, driver::getLeftX, driver::getLeftY, driver::getRightX);
        placerDrive = new PlacerDrive(bot.drive, placer::getLeftX, placer::getLeftY, Math.toRadians(90));

        bot.drive.setDefaultCommand(localDrive);

        schedule(new RunCommand(telemetry::update));
    }
}
