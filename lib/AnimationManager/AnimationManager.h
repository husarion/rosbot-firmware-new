#pragma once

#include <mbed.h>
#include "ws2812b-dma-driver.h"

enum WS2812B_Animation : uint8_t
{
    WS2812B_OFF = 0,
    WS2812B_SOLID = 1,
    WS2812B_FADE = 2,
    WS2812B_BLINK = 3,
    WS2812B_RAINBOW = 4
};

typedef struct WS2812B_Animation_State
{
    WS2812B_Animation type;
    Color_t color;
} WS2812B_Animation_State_t;

class AnimationManager
{
    public:
        static AnimationManager * getInstance();
        void init();
        void setFadingAnimation(const Color_t * color);
        void setBlinkingAnimation(const Color_t * color);
        void setSolidColor(const Color_t * color);
        void setRainbowAnimation();
        void enableInterface(bool en);

    private:
        static AnimationManager * _instance;
        AnimationManager()
        :_new_message(false)
        ,_anim_state{WS2812B_OFF,{0,0,0}}
        {}
        void ws2812b_thread_fun();
        void setAnimationInternal(WS2812B_Animation type, const Color_t * color);

        void FadeInFadeOutFrame();
        void HalfBlinkFrame();
        void RainbowFrame();
        static void Wheel(Color_t * c, uint8_t WheelPos);

        volatile bool _new_message;
        WS2812B_Animation_State_t _anim_state;
};