package org.firstinspires.ftc.teamcode.helpers.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

/*
 *                    ________
 *                   / ______ \
 *     ------------.-'   _  '-..+              Front of Bot
 *              /   _  ( Y )  _  \                  ^
 *             |  ( X )  _  ( B ) |     Front Left   \    Front Right
 *        ___  '.      ( A )     /|       Wheel       \      Wheel
 *      .'    '.    '-._____.-'  .'       (x/▢)        \     (Y/Δ)
 *     |       |                 |                      \
 *      '.___.' '.               |          Rear Left    \   Rear Right
 *               '.             /             Wheel       \    Wheel
 *                \.          .'              (A/X)        \   (B/O)
 */
public class GamepadEx {
    public class Button {
        private boolean isPressed = false;
        private boolean justPressed = false;
        private Runnable pressFunction = () -> {
        };
        private Runnable releaseFunction = () -> {
        };
        private Runnable holdFunction = () -> {

        };

        public Button(boolean button) {
            isPressed = button;
        }

        public void poll(boolean button) {
            justPressed = isPressed;
            isPressed = button;

            if (isPressed() && !justPressed()) {
                pressFunction.run();
            }
            if (isReleasing()) {
                releaseFunction.run();
            }
            if (isHolding()) {
                holdFunction.run();
            }
        }

        public boolean isPressed() {
            return isPressed;
        }

        public boolean justPressed() {
            return justPressed;
        }

        public boolean isHolding() {
            return isPressed() && justPressed();
        }

        public boolean isReleasing() {
            return !isPressed() && justPressed();
        }

        public void onPress(Runnable func) {
            pressFunction = func;
        }

        public void onRelease(Runnable func) {
            releaseFunction = func;
        }

        public void onHold(Runnable func) {
            holdFunction = func;
        }
    }

    public class ButtonGroup {
        public Button Triangle;
        public Button Square;
        public Button Circle;
        public Button X;

        public ButtonGroup(Gamepad gamepad) {
            Triangle = new Button(gamepad.y);
            Square = new Button(gamepad.x);
            Circle = new Button(gamepad.b);
            X = new Button(gamepad.a);
        }

        public void poll(Gamepad gamepad) {
            Triangle.poll(gamepad.y);
            Square.poll(gamepad.x);
            Circle.poll(gamepad.b);
            X.poll(gamepad.a);
        }
    }

    public ButtonGroup Buttons;

    public GamepadEx(Gamepad gamepad) {
        Buttons = new ButtonGroup(gamepad);
    }

    public void poll(Gamepad gamepad) {
        Buttons.poll(gamepad);
    }
}
