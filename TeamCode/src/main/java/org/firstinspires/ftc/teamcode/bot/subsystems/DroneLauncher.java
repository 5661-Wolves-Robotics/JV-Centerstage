package org.firstinspires.ftc.teamcode.bot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DroneLauncher extends SubsystemBase {

    private final Servo m_servo;

    public enum LauncherState{
        LOADED(0.5),
        FIRED(0.0);

        private final double pos;
        LauncherState(double pos){
            this.pos = pos;
        }

        double getValue(){
            return pos;
        }
    }

    public DroneLauncher(HardwareMap hardwareMap, String servo){
        m_servo = hardwareMap.get(Servo.class, servo);
        m_servo.setPosition(LauncherState.LOADED.getValue());
    }

    public void launch(){
        m_servo.setPosition(LauncherState.FIRED.getValue());
    }

}
