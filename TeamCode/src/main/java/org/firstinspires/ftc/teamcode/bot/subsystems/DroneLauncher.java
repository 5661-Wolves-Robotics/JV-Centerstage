package org.firstinspires.ftc.teamcode.bot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.servo.ToggleServo;

public class DroneLauncher extends SubsystemBase {

    private final ToggleServo m_servo;

    public DroneLauncher(HardwareMap hardwareMap, String servo){
        m_servo = new ToggleServo(hardwareMap.get(Servo.class, servo), 0.5 /*LOADED*/, 0.0 /*FIRED*/, true, true);
    }

    public void launch(){
        m_servo.setState(false);
    }

}
