#ifndef __OLED_H
#define __OLED_H

enum OLED_MODE
{
    OLED_8x16 = 1,
    OLED_8x6,
    OLED_12x6,
    OLED_24x12
};

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char,enum OLED_MODE Mode);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String, enum OLED_MODE Mode);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length, enum OLED_MODE Mode);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length, enum OLED_MODE Mode);
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length, enum OLED_MODE Mode);
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length, enum OLED_MODE Mode);

#endif
