package org.firstinspires.ftc.teamcode.bot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake extends SubsystemBase {

    private final Servo m_servo;
    private final DcMotorEx m_motor;

    private final double POWER = 0.7;

    public enum IntakeState{
        RAISED(0.66),
        LOWERED(0.23);

        private final double pos;
        IntakeState(double pos){
            this.pos = pos;
        }

        double getValue(){
            return pos;
        }
    }

    private IntakeState state = IntakeState.RAISED;

    public Intake(HardwareMap hardwareMap, String servoName, String motorName){
        m_servo = hardwareMap.get(Servo.class, servoName);
        m_motor = hardwareMap.get(DcMotorEx.class, motorName);

        setState(state);
    }

    public void setState(IntakeState newState){
        state = newState;
        m_servo.setPosition(state.getValue());
    }

    public void reverse(){
        m_motor.setPower(-POWER);
    }

    public void power(){
        m_motor.setPower(POWER);
    }

    public void stop(){
        m_motor.setPower(0);
    }

    public IntakeState getState(){
        return state;
    }

}
