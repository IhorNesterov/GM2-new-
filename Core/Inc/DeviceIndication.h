#pragma once
#include <stm32f1xx.h>

typedef struct PIN_t
{
    GPIO_TypeDef* port;
    uint16_t pin;    
}Pin;

typedef struct Led_t
{
    Pin* pin;
    bool state;
}Led;

typedef struct GM_Indication_t
{
    Led* normalLed;
    Led* rxLed;
    Led* txLed;
    Led* tickLed;
    Led* watchDogLed;
    Led* problemLed;

    Led* normalRelay;
    Led* firstDangerRelay;
    Led* secondDangerRelay;
}GM_Indication;

void Led_On(Led* led)
{
    HAL_GPIO_WritePin(led->pin->port,led->pin->pin,GPIO_PIN_SET);
    led->state = true;
}

void Led_Off(Led* led)
{
    HAL_GPIO_WritePin(led->pin->port,led->pin->pin,GPIO_PIN_RESET);
    led->state = false;
}

void Led_Toggle(Led* led)
{
    if(!led->state)
    {
        Led_On(led);
    }
    else
    {
        Led_Off(led);
    }
}

void InitDeviceIndication(GM_Indication* indication,Led* normal,Led* tx,Led* rx,Led* tick,Led* watchDog,Led* problem,Led* firstR,Led* secondR,Led* normalR)
{
    indication->normalLed = normal;
    indication->rxLed = rx;
    indication->txLed = tx;
    indication->problemLed = problem;
    indication->watchDogLed = watchDog;

    indication->normalRelay = normalR;
    indication->firstDangerRelay = firstR;
    indication->secondDangerRelay = secondR;
}



