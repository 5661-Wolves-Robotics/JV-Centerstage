package org.firstinspires.ftc.teamcode.bot;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.bot.subsystems.MecanumDriveBase;

@Config
public class CenterStageBot extends Robot {

    public MecanumDriveBase drive;

    public CenterStageBot(HardwareMap hardwareMap){
        drive = new MecanumDriveBase(hardwareMap);

        register(drive);
    }

}
