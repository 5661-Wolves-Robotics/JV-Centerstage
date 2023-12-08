package org.firstinspires.ftc.teamcode.bot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawArm extends SubsystemBase {

    private final Servo m_armServo;
    private final Servo m_clawServo;

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

    public enum ClawState{
        CLOSED(0.0),
        OPEN(0.21);

        private final double pos;
        ClawState(double pos){
            this.pos = pos;
        }

        double getValue(){
            return pos;
        }
    }

    private ClawState clawState = ClawState.OPEN;

    public ClawArm(HardwareMap hardwareMap, String armServo, String clawServo){
        m_armServo = hardwareMap.get(Servo.class, armServo);
        m_clawServo = hardwareMap.get(Servo.class, clawServo);

        setArmState(armState);
        setClawState(clawState);
    }

    public void setArmState(ArmState state){
        armState = state;
        m_armServo.setPosition(state.getValue());
    }

    public ArmState getArmState(){
        return armState;
    }

    public void setClawState(ClawState state){
        clawState = state;
        m_clawServo.setPosition(state.getValue());
    }

    public ClawState getClawState() {
        return clawState;
    }

    public void open(){
        setClawState(ClawState.OPEN);
    }

    public void close(){
        setClawState(ClawState.CLOSED);
    }

}
