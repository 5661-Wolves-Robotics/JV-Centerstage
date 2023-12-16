package org.firstinspires.ftc.teamcode.bot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.servo.ToggleServo;

public class ClawArm extends SubsystemBase {

    private final Servo m_armServo;
    private final ToggleServo m_clawServo;

    public enum ArmState{
        RAISED(0.3),
        STORED(0.22),
        LOWERED(0.16);

        private final double pos;
        ArmState(double pos){
            this.pos = pos;
        }

        double getValue(){
            return pos;
        }
    }

    private ArmState armState = ArmState.STORED;

    public ClawArm(HardwareMap hardwareMap, String armServo, String clawServo){
        m_armServo = hardwareMap.get(Servo.class, armServo);
        m_clawServo = new ToggleServo(hardwareMap.get(Servo.class, clawServo), 0.21 /*OPEN*/, 0. /*CLOSED*/, true, true);

        setArmState(armState);
    }

    public void setArmState(ArmState state){
        armState = state;
        m_armServo.setPosition(state.getValue());
    }

    public ArmState getArmState(){
        return armState;
    }

    public void open(){
        m_clawServo.setState(true);
    }

    public void close(){
        m_clawServo.setState(false);
    }

    public void toggleClaw(){
        m_clawServo.toggle();
    }

}
