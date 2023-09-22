/*
*******************************************************************************
* Copyright (c) 2022 by M5Stack
*                  Equipped with M5AtomS3 sample source code
*                          配套  M5AtomS3 示例源代码
* Visit for more information: https://docs.m5stack.com/en/core/AtomS3
* 获取更多资料请访问: https://docs.m5stack.com/zh_CN/core/AtomS3
*
* Describe: WIFI TCP.
* Date: 2023/1/15
*******************************************************************************
  M5AtomS3 will sends a message to a TCP server
  M5AtomS3 将向TCP服务器发送一条数据
*/

#include <M5AtomS3.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include "Time.h"
#include <base64.h>
#include <sesami_util.h>

// Set the name and password of the wifi to be connected.
// 配置所连接wifi的名称和密码
const char *ssid = "";
const char *password = "";
WiFiMulti WiFiMulti;

String device_uuid = "";
String secret_key = "";
String api_key = "";

void setup()
{
    int sum = 0;
    M5.begin(true, true, false, false);                     // Init M5AtomS3.  初始化M5AtomS3
    WiFiMulti.addAP(ssid, password);                        // Add wifi configuration information.  添加wifi配置信息
    M5.lcd.printf("Waiting connect to WiFi: %s ...", ssid); // Serial port output format string.  串口输出格式化字符串
    while (WiFiMulti.run() != WL_CONNECTED)
    { // If the connection to wifi is not established
      // successfully.  如果没有与wifi成功建立连接
        M5.lcd.print(".");
        delay(1000);
        sum += 1;
        if (sum == 8)
            M5.lcd.print("Conncet failed!");
    }
    M5.lcd.println("\nWiFi connected");
    M5.lcd.print("IP address: ");
    M5.lcd.println(WiFi.localIP()); // The serial port outputs the IP address
                                    // of the M5AtomS3.  串口输出M5AtomS3的IP地址
    delay(500);

    Time.set_time();
}

void loop()
{
    delay(1000);
    get_sesami_status(
        device_uuid,
        api_key);
    USBSerial.println();

    delay(1000);
    get_sesami_history(
        device_uuid,
        api_key);
    USBSerial.println();

    delay(1000);
    operation_sesami(
        device_uuid,
        api_key,
        88,
        secret_key);
    USBSerial.println();
}
