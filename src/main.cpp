#include <Arduino.h>
#include <SPI.h>

#define SD_ENABLE 0

#include "OneButton.h"
#include "Wire.h"
#include "XL9535_driver.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_panel_vendor.h"
#include "ft3267.h"
#include "lvgl.h"
#include "pin_config.h"
#include "WiFi.h"
#include <time.h>
#include <sntp.h>
#include "ui.h"
#if SD_ENABLE
#include "SD_MMC.h"
#endif

#define WIFI_SSID "<WIFI_SSID>"
#define WIFI_PASSWORD "<WiFI_PASSWORD>"

#define FT5x06_ADDR 0x38
#define CST820_ADDR 0x15
#define GT911_ADDR 0x5A

#define MSG_TOUCH_UPDATE 2

const char* ntpServer1 = "de.pool.ntp.org";
const char* ntpServer2 = "ntp.rz.tu-harburg.de";
const long gmtOffset_sec = (1 * 60 * 60);
const int daylightOffset_sec = 0;

const char* weekday_chi_str[] = {
    "SUN",
    "MON",
    "TUE",
    "WED",
    "THU",
    "FRI",
    "SAT"
};

typedef struct
{
  uint16_t x;
  uint16_t y;
} touch_point_t;

typedef struct
{
  uint8_t cmd;
  uint8_t data[16];
  uint8_t databytes; // No of data in data; bit 7 = delay after set; 0xFF =
  // end of cmds.
} lcd_init_cmd_t;

static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
static lv_disp_drv_t disp_drv; // contains callback functions
static lv_indev_drv_t indev_drv;

DRAM_ATTR static const lcd_init_cmd_t st_init_cmds[] = {
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x10}, 0x05},
    {0xC0, {0x3b, 0x00}, 0x02},
    {0xC1, {0x0b, 0x02}, 0x02},
    {0xC2, {0x07, 0x02}, 0x02},
    {0xCC, {0x10}, 0x01},
    {0xCD, {0x08}, 0x01}, // 用565时屏蔽    666打开
    {0xb0,
     {0x00, 0x11, 0x16, 0x0e, 0x11, 0x06, 0x05, 0x09, 0x08, 0x21, 0x06, 0x13,
      0x10, 0x29, 0x31, 0x18},
     0x10},
    {0xb1,
     {0x00, 0x11, 0x16, 0x0e, 0x11, 0x07, 0x05, 0x09, 0x09, 0x21, 0x05, 0x13,
      0x11, 0x2a, 0x31, 0x18},
     0x10},
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x11}, 0x05},
    {0xb0, {0x6d}, 0x01},
    {0xb1, {0x37}, 0x01},
    {0xb2, {0x81}, 0x01},
    {0xb3, {0x80}, 0x01},
    {0xb5, {0x43}, 0x01},
    {0xb7, {0x85}, 0x01},
    {0xb8, {0x20}, 0x01},
    {0xc1, {0x78}, 0x01},
    {0xc2, {0x78}, 0x01},
    {0xc3, {0x8c}, 0x01},
    {0xd0, {0x88}, 0x01},
    {0xe0, {0x00, 0x00, 0x02}, 0x03},
    {0xe1,
     {0x03, 0xa0, 0x00, 0x00, 0x04, 0xa0, 0x00, 0x00, 0x00, 0x20, 0x20},
     0x0b},
    {0xe2,
     {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00},
     0x0d},
    {0xe3, {0x00, 0x00, 0x11, 0x00}, 0x04},
    {0xe4, {0x22, 0x00}, 0x02},
    {0xe5,
     {0x05, 0xec, 0xa0, 0xa0, 0x07, 0xee, 0xa0, 0xa0, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00},
     0x10},
    {0xe6, {0x00, 0x00, 0x11, 0x00}, 0x04},
    {0xe7, {0x22, 0x00}, 0x02},
    {0xe8,
     {0x06, 0xed, 0xa0, 0xa0, 0x08, 0xef, 0xa0, 0xa0, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00},
     0x10},
    {0xeb, {0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0x00}, 0x07},
    {0xed,
     {0xff, 0xff, 0xff, 0xba, 0x0a, 0xbf, 0x45, 0xff, 0xff, 0x54, 0xfb, 0xa0,
      0xab, 0xff, 0xff, 0xff},
     0x10},
    {0xef, {0x10, 0x0d, 0x04, 0x08, 0x3f, 0x1f}, 0x06},
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x13}, 0x05},
    {0xef, {0x08}, 0x01},
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x00}, 0x05},
    {0x36, {0x08}, 0x01},
    {0x3a, {0x66}, 0x01},
    {0x11, {0x00}, 0x80},
    // {0xFF, {0x77, 0x01, 0x00, 0x00, 0x12}, 0x05},
    // {0xd1, {0x81}, 0x01},
    // {0xd2, {0x06}, 0x01},
    {0x29, {0x00}, 0x80},
    {0, {0}, 0xff}
};

XL9535 xl;
OneButton button(0, true);

const int backlightPin = EXAMPLE_PIN_NUM_BK_LIGHT;
bool click = false;
bool touchDevicesOnline = false;
uint8_t touchAddress = 0;

static bool ntpGotTime = false;
static struct tm timeinfo;
static unsigned long next_get_timeinfo;
static unsigned long last_second_angle = 999;
static int last_display_mday = -1;
static char str_buf[40]; // sprintf string buffer

static uint8_t curr_anchor_idx = 0;
static int16_t curr_anchor_angle = 0;
static lv_obj_t* anchors[10];
static unsigned long anchor_next_frame_ms;
#define ANCHOR_FPS 30

static void print_chip_info(void)
{
  Serial.print("Chip: ");
  Serial.println(ESP.getChipModel());
  Serial.print("ChipRevision: ");
  Serial.println(ESP.getChipRevision());
  Serial.print("Psram size: ");
  Serial.print(ESP.getPsramSize() / 1024);
  Serial.println("KB");
  Serial.print("Flash size: ");
  Serial.print(ESP.getFlashChipSize() / 1024);
  Serial.println("KB");
  Serial.print("CPU frequency: ");
  Serial.print(ESP.getCpuFreqMHz());
  Serial.println("MHz");
}

#if SD_ENABLE
static void SD_init(void)
{
  xl.digitalWrite(
    SD_CS_PIN,
    1); // To use SDIO one-line mode, you need to pull the CS pin high
  SD_MMC.setPins(SD_CLK_PIN, SD_CMD_PIN, SD_D0_PIN);
  if (!SD_MMC.begin("/sdcard", true, true))
  {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE)
  {
    Serial.println("No SD card attached");
    return;
  }
  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC)
    Serial.println("MMC");
  else if (cardType == CARD_SD)
    Serial.println("SDSC");
  else if (cardType == CARD_SDHC)
    Serial.println("SDHC");
  else
    Serial.println("UNKNOWN");
  uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
}
#endif

static void scanDevices(void)
{
  byte error, address;
  int nDevices = 0;
  Serial.println("Scanning for I2C devices ...");
  for (address = 0x01; address < 0x7f; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.printf("I2C device found at address 0x%02X\n", address);
      if (address == FT5x06_ADDR)
      {
        Serial.println("Find FT5X06 touch device!");
        touchDevicesOnline = true;
        touchAddress = FT5x06_ADDR;
      }
      else if (address == CST820_ADDR)
      {
        Serial.println("Find CST820 touch device!");
        touchDevicesOnline = true;
        touchAddress = CST820_ADDR;
      }
      else if (address == GT911_ADDR)
      {
        Serial.println("Find GT911 touch device!");
        touchDevicesOnline = true;
        touchAddress = GT911_ADDR;
      }
      nDevices++;
    }
    else if (error != 2)
    {
      Serial.printf("Error %d at address 0x%02X\n", error, address);
    }
  }
  if (nDevices == 0)
  {
    Serial.println("No I2C devices found");
  }
}

static void lcd_send_data(uint8_t data)
{
  uint8_t n;
  for (n = 0; n < 8; n++)
  {
    if (data & 0x80)
      xl.digitalWrite(LCD_SDA_PIN, 1);
    else
      xl.digitalWrite(LCD_SDA_PIN, 0);

    data <<= 1;
    xl.digitalWrite(LCD_CLK_PIN, 0);
    xl.digitalWrite(LCD_CLK_PIN, 1);
  }
}

static void lcd_cmd(const uint8_t cmd)
{
  xl.digitalWrite(LCD_CS_PIN, 0);
  xl.digitalWrite(LCD_SDA_PIN, 0);
  xl.digitalWrite(LCD_CLK_PIN, 0);
  xl.digitalWrite(LCD_CLK_PIN, 1);
  lcd_send_data(cmd);
  xl.digitalWrite(LCD_CS_PIN, 1);
}

static void lcd_data(const uint8_t* data, int len)
{
  uint32_t i = 0;
  if (len == 0)
    return; // no need to send anything
  do
  {
    xl.digitalWrite(LCD_CS_PIN, 0);
    xl.digitalWrite(LCD_SDA_PIN, 1);
    xl.digitalWrite(LCD_CLK_PIN, 0);
    xl.digitalWrite(LCD_CLK_PIN, 1);
    lcd_send_data(*(data + i));
    xl.digitalWrite(LCD_CS_PIN, 1);
    i++;
  } while (len--);
}

static void tft_init(void)
{
  xl.digitalWrite(LCD_CS_PIN, 1);
  xl.digitalWrite(LCD_SDA_PIN, 1);
  xl.digitalWrite(LCD_CLK_PIN, 1);

  // Reset the display
  xl.digitalWrite(LCD_RST_PIN, 1);
  delay(200);
  xl.digitalWrite(LCD_RST_PIN, 0);
  delay(200);
  xl.digitalWrite(LCD_RST_PIN, 1);
  delay(200);
  int cmd = 0;
  while (st_init_cmds[cmd].databytes != 0xff)
  {
    lcd_cmd(st_init_cmds[cmd].cmd);
    lcd_data(st_init_cmds[cmd].data, st_init_cmds[cmd].databytes & 0x1F);
    if (st_init_cmds[cmd].databytes & 0x80)
    {
      delay(100);
    }
    cmd++;
  }
  Serial.println("Register setup complete");
}

static void lvgl_flush_cb(lv_disp_drv_t* drv, const lv_area_t* area,
  lv_color_t* color_map)
{
  esp_lcd_panel_handle_t panel_handle =
    (esp_lcd_panel_handle_t)drv->user_data;
  int offsetx1 = area->x1;
  int offsetx2 = area->x2;
  int offsety1 = area->y1;
  int offsety2 = area->y2;
  esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1,
    offsety2 + 1, color_map);
  lv_disp_flush_ready(drv);
}

static void lv_touchpad_read(lv_indev_drv_t* indev_driver,
  lv_indev_data_t* data)
{
  static uint16_t lastX, lastY;
  touch_point_t p = { 0 };
  uint8_t touch_points_num;
  ft3267_read_pos(&touch_points_num, &p.x, &p.y);
  data->point.x = p.x;
  data->point.y = p.y;
  if (p.x != lastX || p.y != lastY)
  {
    lastX = p.x;
    lastY = p.y;
    data->state = LV_INDEV_STATE_PR;
  }
  else
  {
    data->state = LV_INDEV_STATE_REL;
  }
  lv_msg_send(MSG_TOUCH_UPDATE, &p);
}

static void display_init()
{
  esp_lcd_panel_handle_t panel_handle = NULL;

  scanDevices();
  ft3267_init(Wire);

  tft_init();

  esp_lcd_rgb_panel_config_t panel_config = {
      .clk_src = LCD_CLK_SRC_PLL160M,
      .timings =
          {
              .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
              .h_res = EXAMPLE_LCD_H_RES,
              .v_res = EXAMPLE_LCD_V_RES,
              // The following parameters should refer to LCD spec
              .hsync_pulse_width = 1,
              .hsync_back_porch = 30,
              .hsync_front_porch = 50,
              .vsync_pulse_width = 1,
              .vsync_back_porch = 30,
              .vsync_front_porch = 20,
              .flags =
                  {
                      .pclk_active_neg = 1,
                  },
          },
      .data_width = 16, // RGB565 in parallel mode, thus 16bit in width
      .psram_trans_align = 64,
      .hsync_gpio_num = EXAMPLE_PIN_NUM_HSYNC,
      .vsync_gpio_num = EXAMPLE_PIN_NUM_VSYNC,
      .de_gpio_num = EXAMPLE_PIN_NUM_DE,
      .pclk_gpio_num = EXAMPLE_PIN_NUM_PCLK,
      .data_gpio_nums =
          {
      // EXAMPLE_PIN_NUM_DATA0,
      EXAMPLE_PIN_NUM_DATA13,
      EXAMPLE_PIN_NUM_DATA14,
      EXAMPLE_PIN_NUM_DATA15,
      EXAMPLE_PIN_NUM_DATA16,
      EXAMPLE_PIN_NUM_DATA17,

      EXAMPLE_PIN_NUM_DATA6,
      EXAMPLE_PIN_NUM_DATA7,
      EXAMPLE_PIN_NUM_DATA8,
      EXAMPLE_PIN_NUM_DATA9,
      EXAMPLE_PIN_NUM_DATA10,
      EXAMPLE_PIN_NUM_DATA11,
      // EXAMPLE_PIN_NUM_DATA12,

      EXAMPLE_PIN_NUM_DATA1,
      EXAMPLE_PIN_NUM_DATA2,
      EXAMPLE_PIN_NUM_DATA3,
      EXAMPLE_PIN_NUM_DATA4,
      EXAMPLE_PIN_NUM_DATA5,
  },
.disp_gpio_num = EXAMPLE_PIN_NUM_DISP_EN,
.on_frame_trans_done = NULL,
.user_ctx = NULL,
.flags =
    {
        .fb_in_psram = 1, // allocate frame buffer in PSRAM
    },
  };
  ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

  lv_init();
  // alloc draw buffers used by LVGL from PSRAM
  lv_color_t* buf1 = (lv_color_t*)ps_malloc(
    EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(lv_color_t));
  assert(buf1);
  lv_color_t* buf2 = (lv_color_t*)ps_malloc(
    EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(lv_color_t));
  assert(buf2);
  lv_disp_draw_buf_init(&disp_buf, buf1, buf2,
    EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES);

  Serial.println("Register display driver to LVGL");
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = EXAMPLE_LCD_H_RES;
  disp_drv.ver_res = EXAMPLE_LCD_V_RES;
  disp_drv.flush_cb = lvgl_flush_cb;
  disp_drv.draw_buf = &disp_buf;
  disp_drv.user_data = panel_handle;
  lv_disp_t* disp = lv_disp_drv_register(&disp_drv);

  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = lv_touchpad_read;
  lv_indev_drv_register(&indev_drv);
}

static void waitInterruptReady()
{
  Serial.println("waitInterruptReady ...");
  uint32_t timeout = millis() + 500;
  while (!digitalRead(TP_INT_PIN))
  {
    delay(10);
  }
}

// LilyGo  T-RGB  control backlight chip has 16 levels of adjustment range
// The adjustable range is 0~15, 0 is the minimum brightness, 15 is the maximum
// brightness
static void setBrightness(uint8_t value)
{
  static uint8_t level = 0;
  static uint8_t steps = 16;
  if (value == 0)
  {
    digitalWrite(backlightPin, 0);
    delay(3);
    level = 0;
    return;
  }
  if (level == 0)
  {
    digitalWrite(backlightPin, 1);
    level = steps;
    delayMicroseconds(30);
  }
  int from = steps - level;
  int to = steps - value;
  int num = (steps + to - from) % steps;
  for (int i = 0; i < num; i++)
  {
    digitalWrite(backlightPin, 0);
    digitalWrite(backlightPin, 1);
  }
  level = value;
}

void calculate_next_get_timeinfo()
{
  getLocalTime(&timeinfo);
  next_get_timeinfo = ((millis() / 1000) + 1 + (60 - timeinfo.tm_sec)) * 1000;
}

// Callback function (get's called when time adjusts via NTP)
void timeavailable(struct timeval* t)
{
  Serial.println("Got time adjustment from NTP!");

  ntpGotTime = true;
  calculate_next_get_timeinfo();

  lv_obj_clear_flag(ui_ImageArmSecond, LV_OBJ_FLAG_HIDDEN);
  lv_obj_clear_flag(ui_ImageArmMinute, LV_OBJ_FLAG_HIDDEN);
  lv_obj_clear_flag(ui_ImageArmHour, LV_OBJ_FLAG_HIDDEN);
  lv_obj_clear_flag(ui_LabelWeekday, LV_OBJ_FLAG_HIDDEN);
  lv_obj_clear_flag(ui_LabelDate, LV_OBJ_FLAG_HIDDEN);
}

void setup()
{
  Serial.begin(115200);
  Wire.begin(IIC_SDA_PIN, IIC_SCL_PIN);

  xl.begin();

  uint8_t pin = (1 << PWR_EN_PIN) | (1 << LCD_CS_PIN) | (1 << TP_RES_PIN) |
    (1 << LCD_SDA_PIN) | (1 << LCD_CLK_PIN) | (1 << LCD_RST_PIN) |
    (1 << SD_CS_PIN);

  xl.pinMode8(0, pin, OUTPUT);
  xl.digitalWrite(PWR_EN_PIN, HIGH);

  print_chip_info();

#if SD_ENABLE
  SD_init();
#endif

  delay(100);
  xl.digitalWrite(TP_RES_PIN, LOW);
  delay(300);
  xl.digitalWrite(TP_RES_PIN, HIGH);
  delay(300);
  pinMode(TP_INT_PIN, INPUT);

  display_init();

  button.attachClick(
    []()
    {
      Serial.println("Button Clicked!!!");
      click = true;
    });
  waitInterruptReady();
  lv_task_handler();

  pinMode(backlightPin, OUTPUT);
  // LilyGo T-RGB control backlight chip has 16 levels of adjustment range
  for (int i = 0; i < 16; ++i)
  {
    setBrightness(i);
    delay(30);
  }

  sntp_set_time_sync_notification_cb(timeavailable);
  sntp_servermode_dhcp(1);
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);

  // Init WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  //Initialize UI
  ui_init();

  /* assign images to array after ui_init() */
  anchors[0] = ui_Anchor0;
  anchors[1] = ui_Anchor1;
  anchors[2] = ui_Anchor2;
  anchors[3] = ui_Anchor3;
  anchors[4] = ui_Anchor4;
  anchors[5] = ui_Anchor5;
  anchors[6] = ui_Anchor6;
  anchors[7] = ui_Anchor7;
  anchors[8] = ui_Anchor8;
  anchors[9] = ui_Anchor9;
  anchor_next_frame_ms = millis() + (1000 / ANCHOR_FPS);
}

void loop()
{

  if (click || !digitalRead(TP_INT_PIN))
  {
    click = false;
    // TODO: WIP
  }
  button.tick();
  lv_task_handler();

  unsigned long ms = millis();
  // handle ms overflow, over 49.71 days
  if (ms < 200)
  {
    anchor_next_frame_ms = ms;
    calculate_next_get_timeinfo();
  }

  // handle anchor escapement movement
  if (ms >= anchor_next_frame_ms)
  {
    uint8_t next_anchor_idx = curr_anchor_idx + 1;
    if (next_anchor_idx > 9)
    {
      next_anchor_idx = 0;
      // rotate 2.4 degrees for each 10 images loop
      curr_anchor_angle += 24;

      if (curr_anchor_angle >= 3600)
      {
        curr_anchor_angle = 0;
      }
    }

    // rotate and show next image then hide current image
    lv_img_set_angle(anchors[next_anchor_idx], curr_anchor_angle);
    lv_obj_clear_flag(anchors[next_anchor_idx], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(anchors[curr_anchor_idx], LV_OBJ_FLAG_HIDDEN);

    curr_anchor_idx = next_anchor_idx;
    anchor_next_frame_ms += (1000 / ANCHOR_FPS);
  }

  if (ntpGotTime)
  {
    // call getLocalTime() every minute
    if (ms >= next_get_timeinfo)
    {
      getLocalTime(&timeinfo);
      // detect time drifted
      if (timeinfo.tm_sec > 1)
      {
        calculate_next_get_timeinfo();
      }
      else
      {
        next_get_timeinfo += 60000;
      }
    }

    // set watch arms' angle
    int16_t angle = (millis() + 60000 - next_get_timeinfo) * 3600 / 60000;
    if (last_second_angle != angle)
    {
      lv_img_set_angle(ui_ImageArmSecond, angle);
      angle = (angle + (timeinfo.tm_min * 3600)) / 60;
      lv_img_set_angle(ui_ImageArmMinute, angle);
      angle = (angle + (timeinfo.tm_hour * 3600)) / 12;
      lv_img_set_angle(ui_ImageArmHour, angle);

      last_second_angle = angle;
    }

    // set labels' text
    if (last_display_mday != timeinfo.tm_mday)
    {
      sprintf(str_buf, "%d", timeinfo.tm_mday);
      lv_label_set_text(ui_LabelDate, str_buf);
      lv_label_set_text(ui_LabelWeekday, weekday_chi_str[timeinfo.tm_wday]);
      last_display_mday = timeinfo.tm_mday;
    }
  }
}