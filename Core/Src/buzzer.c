#include "buzzer.h"
#include "delay.h"


void Buzzer_KeySound(void)
{

  unsigned int m=30;//60;//70;//80
	while(m--){

    HAL_GPIO_TogglePin(BUZZER_GPIO,BUZZER_PIN);//BEEP=!BEEP;
    //delay_us(600);
    delay_us(500);//500//__delay_us(300);;//__delay_us(800);//delayUS(300);
   
    }
       
}



