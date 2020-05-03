/*
 * Teensy 4.0 with LittlevGL Graphic library on a 800 x 480 RA8875 LCD
 * 
 * May 2020
 * skpang.co.uk
 * 
  MIT License
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
 */

#include <lvgl.h>
#include <SPI.h>
#include <RA8875.h>
#include <Wire.h>

#include <GSL1680.h>   // Capacitive touch screen

#define RA8875_CS 10 
#define RA8875_RESET 9//any pin or 255 to disable it!

RA8875 display = RA8875(RA8875_CS, RA8875_RESET);

#define LVGL_TICK_PERIOD 10
// GLS1680 Capacitive touch Pins
#define WAKE 4
#define INTRPT 15

GSL1680 TS = GSL1680();
int screenWidth = 800;
int screenHeight = 480;
int oldTouchX = 0;
int oldTouchY = 0;

volatile uint32_t count = 0;
IntervalTimer TX_timer;

static lv_disp_buf_t disp_buf;
static lv_color_t buf[LV_HOR_RES_MAX * 40];

int led = 13;

IntervalTimer tick;
static void lv_tick_handler(void)
{

  lv_tick_inc(LVGL_TICK_PERIOD);
}

bool my_touchpad_read(lv_indev_drv_t * indev_driver, lv_indev_data_t * data)
{
    uint16_t touchX, touchY;
 
    if(digitalRead(INTRPT) == HIGH) 
    {
        Serial.println("Touched");
        int NBFinger = TS.dataread();
        for(int i=0; i<NBFinger; i++)
        {
             touchX = TS.readFingerX(i);
             touchY = TS.readFingerY(i);
        
        }
        
        if ((touchX != oldTouchX) || (touchY != oldTouchY))
        {
             Serial.print(touchX);
              Serial.print(" ");
              Serial.println(touchY);
              oldTouchY = touchY;
              oldTouchX = touchX;
              data->state = LV_INDEV_STATE_PR; 
             //  data->state = touched ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL; 
            
                //Save the state and save the pressed coordinate
              //  if(data->state == LV_INDEV_STATE_PR) touchpad_get_xy(&last_x, &last_y);
               
                //Set the coordinates (if released use the last pressed coordinates)
                data->point.x = touchX;
                data->point.y = touchY;
    
        }

    }
     
    return 0;
}

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{ 

  display.writeRect(area->x1, area->y1, area->x2 - area->x1 +1, area->y2 - area->y1+1, (uint16_t *)color_p);

  lv_disp_flush_ready(disp); /* tell lvgl that flushing is done */
  
}


static void event_handler(lv_obj_t * obj, lv_event_t event)
{
 
    if(event == LV_EVENT_CLICKED) {
        Serial.printf("Clicked\n");
    }
    else if(event == LV_EVENT_VALUE_CHANGED) {
       Serial.printf("Toggled\n");
    }
}
// -------------------------------------------------------------
void setup(void)
{
 
  pinMode(led,OUTPUT);
  digitalWrite(led,HIGH);
  pinMode(INTRPT,INPUT);
 //  delay(800);
  
  TS.begin(WAKE, INTRPT);                 // Startup sequence CONTROLER part for touch screen
  Serial.println(F("Teensy 4.0 LitlevGL 800x480 Button test. skpang.co.uk"));
  
  display.begin(RA8875_800x480);
  display.fillWindow(RA8875_GREEN);
  
  lv_init();
  lv_disp_buf_init(&disp_buf, buf, NULL, LV_HOR_RES_MAX * 40);
  
 /*Initialize the display*/
  lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.buffer = &disp_buf;
  lv_disp_drv_register(&disp_drv);

  lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);             /*Descriptor of a input device driver*/
  indev_drv.type = LV_INDEV_TYPE_POINTER;    /*Touch pad is a pointer-like device*/
  indev_drv.read_cb = my_touchpad_read;      /*Set your driver function*/
  lv_indev_drv_register(&indev_drv);         /*Finally register the driver*/

  lv_ex_btn_1();
  lv_obj_t *label = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_text(label, "Hello Teensy 4.0 LittlevGL on RA8875 skpang.co.uk");
  lv_obj_align(label, NULL, LV_ALIGN_CENTER, 0, -85);

  tick.begin(lv_tick_handler, LVGL_TICK_PERIOD * 1000);  // Start tick timer
  Serial.println("tick.begin");
}

void printEvent(String Event, lv_event_t event)
{
  
  Serial.print(Event);
  printf(" ");

  switch(event) {
      case LV_EVENT_PRESSED:
          printf("Pressed\n");
          break;

      case LV_EVENT_SHORT_CLICKED:
          printf("Short clicked\n");
          break;

      case LV_EVENT_CLICKED:
          printf("Clicked\n");
          break;

      case LV_EVENT_LONG_PRESSED:
          printf("Long press\n");
          break;

      case LV_EVENT_LONG_PRESSED_REPEAT:
          printf("Long press repeat\n");
          break;

      case LV_EVENT_RELEASED:
          printf("Released\n");
          break;
  }
}

void lv_ex_btn_1(void)
{
    lv_obj_t * label;

    lv_obj_t * btn1 = lv_btn_create(lv_scr_act(), NULL);
    lv_obj_set_event_cb(btn1, event_handler);
    lv_obj_align(btn1, NULL, LV_ALIGN_CENTER, 0, -40);

    label = lv_label_create(btn1, NULL);
    lv_label_set_text(label, "Button");

    lv_obj_t * btn2 = lv_btn_create(lv_scr_act(), NULL);
    lv_obj_set_event_cb(btn2, event_handler);
    lv_obj_align(btn2, NULL, LV_ALIGN_CENTER, 0, 40);
    lv_btn_set_toggle(btn2, true);
    lv_btn_toggle(btn2);
    lv_btn_set_fit2(btn2, LV_FIT_NONE, LV_FIT_TIGHT);

    label = lv_label_create(btn2, NULL);
    lv_label_set_text(label, "Toggled");
}
// -------------------------------------------------------------
void loop(void)
{
  
  lv_task_handler(); /* let the GUI do its work */
  delay(5);
  
} // Main loop
