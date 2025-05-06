//
// Created by giangvu on 5/6/25.
//

#ifndef MOTEUR_H
#define MOTEUR_H

#include "gpio.h"
#include <stdbool.h>
#include "PinNames.h"
#include "stm32_periph.h"
#include "stm32f4xx_gpio_AF.h"

#define FORWARD 0
#define BACKWARD 1
#define TURNED_LEFT 0
#define TURNED_RIGHT 1
#define SPEED_ALPHA 0.1;

extern uint8_t robot_speed_right;
extern uint8_t robot_speed_left;
extern uint8_t robot_direction;
extern uint8_t robot_direction;
extern uint8_t robot_speed;

void initPWM();
void setPWMParameters(unsigned char rightspeed,unsigned char leftspeed,unsigned char direction);
void setSpeed(unsigned char speed);
void goForward(void);
void goBackward(void);
void stop(void);
void turnLeft(void);
void turnRight(void);
void goStraight(void);



#endif //MOTEUR_H
