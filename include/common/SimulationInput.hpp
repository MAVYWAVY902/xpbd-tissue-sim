#ifndef __SIMULATION_INPUT_HPP
#define __SIMULATION_INPUT_HPP

#include <map>
#include <string>

struct SimulationInput
{
    enum class Device
    {
        KEYBOARD=0,
        MOUSE,
        HAPTIC
    };

    enum class Key
    {
        UNKNOWN=-1,
        A,
        B,
        C,
        D,
        E,
        F,
        G,
        H,
        I,
        J,
        K,
        L,
        M,
        N,
        O,
        P,
        Q,
        R,
        S,
        T,
        U,
        V,
        W,
        X,
        Y,
        Z,
        ONE,
        TWO,
        THREE,
        FOUR,
        FIVE,
        SIX,
        SEVEN,
        EIGHT,
        NINE,
        ZERO,
        F1,
        F2,
        F3,
        F4,
        F5,
        F6,
        F7,
        F8,
        F9,
        F10,
        F11,
        F12,
        SPACE,
        ENTER,
        BACKSPACE,
        SHIFT,
        CONTROL,
        TAB,
        CAPS,
        ESC,
        ALT,
        UP,
        DOWN,
        LEFT,
        RIGHT,
        LEFT_BRACKET,
        RIGHT_BRACKET,
        BACKSLASH,
        SLASH,
        COMMA,
        PERIOD,
        DASH,
        EQUALS,
        SEMICOLON,
        APOSTROPHE,
        BACKTICK
    };

    enum class KeyAction
    {
        PRESS=0,
        RELEASE
    };

    struct ActionModifier
    {
        constexpr static int NONE=0;
        constexpr static int SHIFT=1;
        constexpr static int CTRL=2;
        constexpr static int ALT=4;
    };

    enum class MouseButton
    {
        LEFT=0,
        RIGHT,
        MIDDLE
    };

    enum class MouseAction
    {
        PRESS=0,
        RELEASE
    };
};

#endif // __SIMULATION_INPUT_HPP