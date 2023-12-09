package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

public class PS4GamepadEx extends GamepadEx {

    /**
     * The constructor, that contains the gamepad object from the
     * opmode.
     *
     * @param gamepad the gamepad object from the opmode
     */
    public PS4GamepadEx(Gamepad gamepad) {
        super(gamepad);
    }

    @Override
    public boolean getButton(GamepadKeys.Button button) {
        boolean buttonValue = false;
        if(gamepad.type() == Gamepad.Type.SONY_PS4){
            switch (button) {
                case A:
                    buttonValue = gamepad.cross;
                    break;
                case B:
                    buttonValue = gamepad.circle;
                    break;
                case X:
                    buttonValue = gamepad.square;
                    break;
                case Y:
                    buttonValue = gamepad.triangle;
                    break;
                case LEFT_BUMPER:
                    buttonValue = gamepad.left_bumper;
                    break;
                case RIGHT_BUMPER:
                    buttonValue = gamepad.right_bumper;
                    break;
                case DPAD_UP:
                    buttonValue = gamepad.dpad_up;
                    break;
                case DPAD_DOWN:
                    buttonValue = gamepad.dpad_down;
                    break;
                case DPAD_LEFT:
                    buttonValue = gamepad.dpad_left;
                    break;
                case DPAD_RIGHT:
                    buttonValue = gamepad.dpad_right;
                    break;
                case BACK:
                    buttonValue = gamepad.back;
                    break;
                case START:
                    buttonValue = gamepad.start;
                    break;
                case LEFT_STICK_BUTTON:
                    buttonValue = gamepad.left_stick_button;
                    break;
                case RIGHT_STICK_BUTTON:
                    buttonValue = gamepad.right_stick_button;
                    break;
                default:
                    buttonValue = false;
                    break;
            }
        } else {
            switch (button) {
                case A:
                    buttonValue = gamepad.a;
                    break;
                case B:
                    buttonValue = gamepad.b;
                    break;
                case X:
                    buttonValue = gamepad.x;
                    break;
                case Y:
                    buttonValue = gamepad.y;
                    break;
                case LEFT_BUMPER:
                    buttonValue = gamepad.left_bumper;
                    break;
                case RIGHT_BUMPER:
                    buttonValue = gamepad.right_bumper;
                    break;
                case DPAD_UP:
                    buttonValue = gamepad.dpad_up;
                    break;
                case DPAD_DOWN:
                    buttonValue = gamepad.dpad_down;
                    break;
                case DPAD_LEFT:
                    buttonValue = gamepad.dpad_left;
                    break;
                case DPAD_RIGHT:
                    buttonValue = gamepad.dpad_right;
                    break;
                case BACK:
                    buttonValue = gamepad.back;
                    break;
                case START:
                    buttonValue = gamepad.start;
                    break;
                case LEFT_STICK_BUTTON:
                    buttonValue = gamepad.left_stick_button;
                    break;
                case RIGHT_STICK_BUTTON:
                    buttonValue = gamepad.right_stick_button;
                    break;
                default:
                    buttonValue = false;
                    break;
            }
        }
        return buttonValue;
    }
}
