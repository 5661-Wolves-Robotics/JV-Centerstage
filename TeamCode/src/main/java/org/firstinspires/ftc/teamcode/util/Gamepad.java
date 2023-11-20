package org.firstinspires.ftc.teamcode.util;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

public class Gamepad{

    public interface ButtonFunc{
        void run();
    }

    public class ButtonCallback{
        Map<ButtonState, ButtonFunc> functions = new HashMap<>();

        public ButtonCallback(ButtonState state, ButtonFunc func){
            functions.put(state, func);
        }

        public void setFunction(ButtonState state, ButtonFunc func){ functions.put(state, func); }
        public void update(ButtonState state){
            ButtonFunc func = functions.get(state);
            if(func != null) func.run();
        }
    }

    com.qualcomm.robotcore.hardware.Gamepad gamepad = null;

    public enum Button
    {
        R_BUMPER,
        L_BUMPER,
        BACK,
        START,
        GUIDE,
        Y,
        X,
        B,
        A,
        DPAD_RIGHT,
        DPAD_LEFT,
        DPAD_DOWN,
        DPAD_UP,
        R_STICK,
        L_STICK,
        TOUCHPAD,
        TOUCHPAD_2,
        TOUCHPAD_1
    }

    public float l_stick_x = 0f;
    public float l_stick_y = 0f;
    public float r_stick_x = 0f;
    public float r_stick_y = 0f;

    public float l_trigger = 0f;
    public float r_trigger = 0f;

    public enum ButtonState
    {
        PRESSED,
        HELD,
        RELEASED,
        INACTIVE
    }

    private Map<Button, ButtonState> buttons = new HashMap<>();
    private Map<Button, ButtonCallback> buttonFuncs =  new HashMap<>();

    public Gamepad(com.qualcomm.robotcore.hardware.Gamepad gamepad) {
        this.gamepad = gamepad;

        for(Button button : Button.values())
        {
            buttons.put(button, ButtonState.RELEASED);
        }
    }

    public void update()
    {
        buttons.put(Button.R_BUMPER, updateState(buttons.get(Button.R_BUMPER), gamepad.right_bumper));
        buttons.put(Button.L_BUMPER, updateState(buttons.get(Button.L_BUMPER), gamepad.left_bumper));
        buttons.put(Button.BACK, updateState(buttons.get(Button.BACK), gamepad.back));
        buttons.put(Button.START, updateState(buttons.get(Button.START), gamepad.start));
        buttons.put(Button.GUIDE, updateState(buttons.get(Button.GUIDE), gamepad.guide));
        buttons.put(Button.Y, updateState(buttons.get(Button.Y), gamepad.y));
        buttons.put(Button.X, updateState(buttons.get(Button.X), gamepad.x));
        buttons.put(Button.B, updateState(buttons.get(Button.B), gamepad.b));
        buttons.put(Button.A, updateState(buttons.get(Button.A), gamepad.a));
        buttons.put(Button.DPAD_RIGHT, updateState(buttons.get(Button.DPAD_RIGHT), gamepad.dpad_right));
        buttons.put(Button.DPAD_LEFT, updateState(buttons.get(Button.DPAD_LEFT), gamepad.dpad_left));
        buttons.put(Button.DPAD_DOWN, updateState(buttons.get(Button.DPAD_DOWN), gamepad.dpad_down));
        buttons.put(Button.DPAD_UP, updateState(buttons.get(Button.DPAD_UP), gamepad.dpad_up));
        buttons.put(Button.R_STICK, updateState(buttons.get(Button.R_STICK), gamepad.right_stick_button));
        buttons.put(Button.L_STICK, updateState(buttons.get(Button.L_STICK), gamepad.left_stick_button));
        buttons.put(Button.TOUCHPAD, updateState(buttons.get(Button.TOUCHPAD), gamepad.touchpad));
        buttons.put(Button.TOUCHPAD_1, updateState(buttons.get(Button.TOUCHPAD_1), gamepad.touchpad_finger_1));
        buttons.put(Button.TOUCHPAD_2, updateState(buttons.get(Button.TOUCHPAD_2), gamepad.touchpad_finger_2));

        l_stick_x = gamepad.left_stick_x;
        l_stick_y = gamepad.left_stick_y;
        r_stick_x = gamepad.right_stick_x;
        r_stick_y = gamepad.right_stick_y;

        r_trigger = gamepad.right_trigger;
        l_trigger = gamepad.left_trigger;

        for(Map.Entry<Button,ButtonCallback> button : buttonFuncs.entrySet()){
            button.getValue().update(getButtonState(button.getKey()));
        }
    }

    public void rumble(int ms)
    {
        gamepad.rumble(ms);
    }

    private ButtonState updateState(ButtonState current, boolean active)
    {
        if(active)
        {
            if(current == ButtonState.INACTIVE || current == ButtonState.RELEASED)
            {
                return ButtonState.PRESSED;
            } else {
                return ButtonState.HELD;
            }
        } else {
            if(current == ButtonState.PRESSED || current == ButtonState.HELD)
            {
                return ButtonState.RELEASED;
            } else {
                return ButtonState.INACTIVE;
            }
        }
    }

    public void bind(Button button, ButtonState state, ButtonFunc func){
        ButtonCallback callback = buttonFuncs.get(button);
        if(callback == null){
            buttonFuncs.put(button, new ButtonCallback(state, func));
        } else {
            callback.setFunction(state, func);
        }
    }

    public ButtonState getButtonState(Button button)
    {
        return buttons.get(button);
    }

    public boolean isActive(Button button){
        return getButtonState(button) == ButtonState.PRESSED || getButtonState(button) == ButtonState.HELD;
    }

}
