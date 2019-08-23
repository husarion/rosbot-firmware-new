#include <mbed.h>
#include "AnimationManager.h"

static Mail<uint32_t, 5> ws2812b_mail_box;
static Thread ws2812b_thread(osPriorityHigh);

// setup servo power DC/DC converter to 5V
static DigitalOut servo_sel1(SERVO_SEL1,0);
static DigitalOut servo_sel2(SERVO_SEL2,0);
static DigitalOut servo_power(SERVO_POWER_ON,0);

AnimationManager * AnimationManager::_instance = NULL;

AnimationManager * AnimationManager::getInstance()
{
    if(_instance==NULL)
    {
        static AnimationManager manager;
        _instance = &manager;
    }
    return _instance;
}

void AnimationManager::init()
{
    ws2812b_thread.start(callback(this,&AnimationManager::ws2812b_thread_fun));
}

void AnimationManager::setAnimationInternal(WS2812B_Animation type, const Color_t * color)
{
    if(ws2812b_mail_box.full())
        return;
    uint32_t * msg = ws2812b_mail_box.alloc();
    if(color != NULL)
        *msg = (uint32_t)(color->r | (color->g << 8) | (color->b << 16) | (type << 24));
    else
        *msg = (uint32_t)(type << 24);
    ws2812b_mail_box.put(msg);
}

void AnimationManager::setFadingAnimation(const Color_t * color)
{
    setAnimationInternal(WS2812B_FADE,color);
}

void AnimationManager::setRainbowAnimation()
{
    setAnimationInternal(WS2812B_RAINBOW,NULL);
}

void AnimationManager::setBlinkingAnimation(const Color_t * color)
{
    setAnimationInternal(WS2812B_BLINK,color);
}

void AnimationManager::setSolidColor(const Color_t * color)
{
    setAnimationInternal(WS2812B_SOLID,color);
}

void AnimationManager::enableInterface(bool en)
{
    if(en)
        setAnimationInternal(WS2812B_SOLID,NULL);
    else
        setAnimationInternal(WS2812B_OFF,NULL);
}

void AnimationManager::FadeInFadeOutFrame()
{
    static int i = 0;
    Color_t color;
    
    if(i>60) i = 0;
    int j = i > 30 ? 60 - i : i;
    i++;

    setColorBrightness(&_anim_state.color,&color,(float)j/30.0f);
    setAll_GRB(&color);
    core_util_critical_section_enter();
    drawFrame();
    core_util_critical_section_exit();
}

void AnimationManager::HalfBlinkFrame()
{
    static int i = 0;
    if(i == 10)
    {
        clearAll();
        setRange_GRB(&_anim_state.color,0,NUM_LEDS/2);
    }
    else if(i == 20)
    {
        clearAll();
        setRange_GRB(&_anim_state.color, NUM_LEDS/2, NUM_LEDS/2);
        i = 0;
    }
    i++;
    
    core_util_critical_section_enter();
    drawFrame();
    core_util_critical_section_exit();
}

void AnimationManager::RainbowFrame()
{
    static int i = 0;
    Color_t color;
    for(int j=0; j < NUM_LEDS; j++)
    {
        Wheel(&color, ((j * 256 / NUM_LEDS) + i) & 0xff);
        setPixel_GRB(&color, j);
    }
    i++;
    
    core_util_critical_section_enter();
    drawFrame();
    core_util_critical_section_exit();
}

void AnimationManager::ws2812b_thread_fun(){

    ws2812b_init();

    while(1)
    {
        osEvent evt = ws2812b_mail_box.get(0);
        if(evt.status == osEventMail)
        {
            uint32_t * message = (uint32_t*)evt.value.p;
            _anim_state.color.r = (uint8_t)(*message & 0xff);
            _anim_state.color.g = (uint8_t)((*message >> 8 ) & 0xff);
            _anim_state.color.b = (uint8_t)((*message >> 16 ) & 0xff);
            _anim_state.type = (WS2812B_Animation)((*message >> 24) & 0xff);
            _new_message = true;            
            ws2812b_mail_box.free(message);
        }

        if(_anim_state.type != WS2812B_OFF && !servo_power.read())
            servo_power = 1;

        switch(_anim_state.type)
        {
            case WS2812B_OFF:
                if(servo_power.read()) servo_power = 0;
                break;
            case WS2812B_SOLID:
                if(_new_message)
                {
                    setAll_GRB(&_anim_state.color);
                    core_util_critical_section_enter();
                    drawFrame();
                    core_util_critical_section_exit();
                    _new_message = false;
                }
                break;
            case WS2812B_FADE:
                FadeInFadeOutFrame();
                break;
            case WS2812B_BLINK:
                HalfBlinkFrame();
                break;
            case WS2812B_RAINBOW:
                RainbowFrame();
                break;
            default:
                break;
        }
        
        ThisThread::sleep_for(50);
    }
}

void AnimationManager::Wheel(Color_t *c, uint8_t WheelPos)
{
    if (WheelPos < 85)
    {
        c->r = WheelPos * 3;
        c->g = 255 - WheelPos * 3;
        c->b = 0;
    }
    else if (WheelPos < 170)
    {
        WheelPos -= 85;
        c->r = 255 - WheelPos * 3;
        c->g = 0;
        c->b = WheelPos * 3;
    }
    else
    {
        WheelPos -= 170;
        c->r = 0;
        c->g = WheelPos * 3;
        c->b = 255 - WheelPos * 3;
    }
}