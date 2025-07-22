/*
 * font.h
 *
 *  Created on: Feb 16, 2025
 *      Author: Lenovo
 */

#ifndef INC_FONT_H_
#define INC_FONT_H_

#ifndef __FONT_H
#define __FONT_H
#include "stdint.h"
#include "string.h"
typedef struct ASCIIFont {
  uint8_t h;
  uint8_t w;
  uint8_t *chars;
} ASCIIFont;

extern const ASCIIFont afont8x6;
extern const ASCIIFont afont12x6;
extern const ASCIIFont afont16x8;
extern const ASCIIFont afont24x12;

/**
 * @brief 字体结构体
 * @note  字库前4字节存储utf8编码 剩余字节存储字模数据
 * @note 字库数据可以使用波特律动LED取模助手生成(https://led.baud-dance.com)
 */
typedef struct Font {
  uint8_t h;              // 字高度
  uint8_t w;              // 字宽度
  const uint8_t *chars;   // 字库 字库前4字节存储utf8编码 剩余字节存储字模数据
  uint8_t len;            // 字库长度 超过256则请改为uint16_t
  const ASCIIFont *ascii; // 缺省ASCII字体 当字库中没有对应字符且需要显示ASCII字符时使用
} Font;

extern const Font font16x16;
extern const Font font12x12;
extern const Font font13x13;
/**
 * @brief 图片结构体
 * @note  图片数据可以使用波特律动LED取模助手生成(https://led.baud-dance.com)
 */


// 声明 Image 结构体（如果 oled.h 未定义）
typedef struct {
    uint16_t w;     // 图片宽度
    uint16_t h;     // 图片高度
    const uint8_t *data; // 图片数据指针
} Image;

// 声明四张图片变量

extern const Image RCBINGLIANImg;;
extern const Image RLBINGLIANImg;
extern const Image RCCHUANLIANImg;
extern const Image RLCHUANLIANImg;
extern const Image CImg;
extern const Image LImg;
extern const Image LEDImg;
extern const Image RImg;
extern const Image LEDYOUImg;

#endif // __FONT_H


#endif /* INC_FONT_H_ */
