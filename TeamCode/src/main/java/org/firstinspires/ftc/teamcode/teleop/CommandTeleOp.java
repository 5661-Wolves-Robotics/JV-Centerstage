package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bot.CenterStageBot;
import org.firstinspires.ftc.teamcode.bot.commands.LocalDrive;
import org.firstinspires.ftc.teamcode.bot.commands.PlacerDrive;
import org.firstinspires.ftc.teamcode.bot.commands.PowerSlide;
import org.firstinspires.ftc.teamcode.bot.commands.ToggleIntake;
import org.firstinspires.ftc.teamcode.bot.subsystems.Intake;

@TeleOp
public class CommandTeleOp extends CommandOpMode {

    CenterStageBot bot;

    GamepadEx driver, placer;
    LocalDrive localDrive;
    PlacerDrive placerDrive;
    PowerSlide powerSlide;
    ToggleIntake toggleIntake;

    @Override
    public void initialize() {
        bot = new CenterStageBot(hardwareMap);

        driver = new GamepadEx(gamepad1);
        placer = new GamepadEx(gamepad2);

        localDrive = new LocalDrive(bot.drive, driver::getLeftX, driver::getLeftY, driver::getRightX);
        placerDrive = new PlacerDrive(bot.drive, placer::getLeftX, placer::getLeftY, Math.toRadians(90));

        powerSlide = new PowerSlide(bot.slide, ()-> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

        toggleIntake = new ToggleIntake(bot.intake);

        driver.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(toggleIntake);
        driver.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(bot.intake::reverse, bot.intake))
                .whenReleased(new ConditionalCommand(
                        new InstantCommand(bot.intake::power, bot.intake),
                        new InstantCommand(bot.intake::stop, bot.intake),
                        ()-> bot.intake.getState() == Intake.IntakeState.LOWERED
                ));
        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new ParallelCommandGroup(
                        placerDrive,
                        new InstantCommand(bot.clawArm::close, bot.clawArm)
                ));

        placer.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(bot.raiseArm);
        placer.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(bot.lowerArm);
        placer.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(bot.droneLauncher::launch, bot.droneLauncher));
        placer.getGamepadButton(GamepadKeys.Button.A)
                .cancelWhenPressed(placerDrive)
                .whenPressed(bot.retractSlide);

        bot.drive.setDefaultCommand(localDrive);
        bot.slide.setDefaultCommand(powerSlide);
    }
}
