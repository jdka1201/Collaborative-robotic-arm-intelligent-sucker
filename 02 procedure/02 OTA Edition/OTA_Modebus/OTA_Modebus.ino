#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Ticker.h>    
#include <EEPROM.h> 
    
#ifdef ESP32
  #include <WiFi.h>
  #include <AsyncTCP.h>
#else
  #include <ESP8266WiFi.h>
  #include <ESPAsyncTCP.h>
#endif
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <ESP8266mDNS.h>


extern "C" {
#include <user_interface.h>
}

#define LocalAddr 0x06				//Modbus地址
#define HostID 0x01   // 本地ID
#define modbus_baud 9600
#define urat_baud 115200
//硬件IO
#define LED_R 2
#define LED_G 12
#define LED_B 13
#define Mode_key 14
#define EN485 16
#define RequestTimeout 500 //默认200（2s

String ToolName = "";//控制器版本 // Air Null Claw
String VController = "V1.0beta";//控制器版本
String VTool = "V1.0beta";//工具版本
String Mode = "SET" ;//模式 RUN
SoftwareSerial Serial485(4, 5); // RX, TX

byte value;

const char* PARAM_INPUT_1 = "input1";
const char* PARAM_INPUT_2 = "input2";
const char* PARAM_INPUT_3 = "input3";
const char* PARAM_INPUT_4 = "direction";

String UserVariables[6];// 0 Air_Max 1 Air_Min 2 Air_TimeOut 3 Claw_Max 4 Claw_Min 5 Claw_TimeOut
String direction;

// int i = 1;
unsigned char uratBuf[10];//自由协议 00：设备类型  01：设备ID 02：功能码 03 04：内容 05 06:CRC   
unsigned char uratsend[5];//功能码 内容*2 crc * 2
unsigned char sendBuf[50];
unsigned char receBuf[50];
unsigned char receCount; //接收数据计数
int count;    // 计数用变量
char TimeOut_Run = 0;//通讯超时启动位
char TimeOut_Pos = 0;//通讯超时判断位
Ticker ticker;// LED time
Ticker TIME_OUT;// LED time

MDNSResponder mdns;
AsyncWebServer server(80);

//读写寄存器 地址 0-8
unsigned int Rs_0;  
unsigned int Rs_1;  
unsigned int Rs_2;
unsigned int Rs_3;
unsigned int Rs_4;
unsigned int Rs_5;
unsigned int Rs_6;
unsigned int Rs_7;
unsigned int Rs_8;
//只读寄存器 地址 10-18
unsigned int R_0;
unsigned int R_1;
unsigned int R_2;
unsigned int R_3;
unsigned int R_4;
unsigned int R_5;
unsigned int R_6;
unsigned int R_7;
unsigned int R_8;
//只写寄存器 地址 20-28
unsigned int S_0;
unsigned int S_1;
unsigned int S_2;
unsigned int S_3;
unsigned int S_4;
unsigned int S_5;
unsigned int S_6;
unsigned int S_7;
unsigned int S_8;
//线圈
unsigned int DO_00001 = 0;
unsigned int DO_00002 = 0;
unsigned int DO_00003 = 0;
unsigned int DO_00004 = 0;
unsigned int DO_00005 = 0;
unsigned int DO_00006 = 0;
unsigned int DO_00007 = 0;
unsigned int DO_00008 = 0;
unsigned int DO_00009 = 0;
unsigned int DO_00010 = 0;
//CRC校验高位字节值表
const unsigned char auchCRCHi[256] = { 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 
0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 
0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40 
} ; 

//CRC校验低位字节值表
const unsigned char auchCRCLo[256] = { 
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 
0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 
0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 
0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4, 
0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 
0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 
0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 
0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 
0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 
0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 
0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 
0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 
0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5, 
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 
0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 
0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 
0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 
0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 
0x43, 0x83, 0x41, 0x81, 0x80, 0x40 
} ;	  

void wificonfig_wifiOn() {
  wifi_fpm_do_wakeup();
  wifi_fpm_close();
  delay(100);
  }

#define FPM_SLEEP_MAX_TIME 0xFFFFFFF


void wificonfig_wifiOff() {
  wifi_station_disconnect();
  wifi_set_opmode(NULL_MODE);
  wifi_set_sleep_type(MODEM_SLEEP_T);
  wifi_fpm_open();
  wifi_fpm_do_sleep(FPM_SLEEP_MAX_TIME);
  delay(100);
  }
// HTML web page to Null_TOOL
const char Null_TOOL[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>Fox_Tool</title>
  <meta charset="utf-8" name="viewport" content="width=device-width, initial-scale=1">
  </head><body>
  <h1 align="center">Fox Tool_Null</h1>
  <h3 align="center">控制器版本：%VController%</h3>
  <h3 align="center">工具版本：Null</h3>
  <p align="center"><a href="/update">固件升级</a></p>
</body>
</html>)rawliteral";
// HTML web page to Air_TOOL  
const char AIR_TOOL[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>Fox_Tool</title>
  <meta charset="utf-8" name="viewport" content="width=device-width, initial-scale=1">
  </head><body>
  <h1 align="center">Fox Tool_Air</h1>
  <form action="/" align="center">
    Max：<input value="%AirMax%" style="width:40px;height:20px" type="text" name="input1">
    <input type="submit" onclick="alert('设置成功!')" value="设置">
  </form><br>
  <form action="/" align="center">
    Min: <input value="%AirMin%" style="width:40px;height:20px" type="text" name="input2">
    <input type="submit" onclick="alert('设置成功!')" value="设置">
  </form><br>
  <form action="/" align="center">
    TimeOut: <input value="%AirTimeOut%" style="width:40px;height:20px" type="text" name="input3">
    <input type="submit" onclick="alert('设置成功!')" value="设置">
  </form><br>
  <form action="/" method="POST" align="center">
     <input type="radio" name="direction" value="stop" checked>
     <label for="CW">停</label>
     <input type="radio" name="direction" value="open">
     <label for="CW">启</label><br>
	<input type="submit" style="width:100px;height:30px" value="Run">
   </form>
  <h3 align="center">控制器版本：%VController%</h3>
  <h3 align="center">工具版本：%VTool%</h3>
  <p align="center"><a href="/update">固件升级</a></p>
</body>
</html>)rawliteral";
// HTML web page to Claw_TOOL  
const char Claw_TOOL[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>Fox_Tool</title>
  <meta charset="utf-8" name="viewport" content="width=device-width, initial-scale=1">
  </head><body>
  <h1 align="center">Fox Tool_Claw</h1>
  <form action="/" align="center">
    Max：<input value="%ClawMax%" style="width:40px;height:20px" type="text" name="input1">
    <input type="submit" onclick="alert('设置成功!')" value="设置">
  </form><br>
  <form action="/" align="center">
    Min: <input value="%ClawMin%" style="width:40px;height:20px" type="text" name="input2">
    <input type="submit" onclick="alert('设置成功!')" value="设置">
  </form><br>
  <form action="/" align="center">
    TimeOut: <input value="%ClawTimeOut%" style="width:40px;height:20px" type="text" name="input3">
    <input type="submit" onclick="alert('设置成功!')" value="设置">
  </form><br>
  <form action="/" method="POST" align="center">
     <input type="radio" name="direction" value="stop" checked>
     <label for="CW">合</label>
     <input type="radio" name="direction" value="open">
     <label for="CW">开</label><br>
	<input type="submit" style="width:100px;height:30px" value="Run">
   </form>
  <h3 align="center">控制器版本：%VController%</h3>
  <h3 align="center">工具版本：%VTool%</h3>
  <p align="center"><a href="/update">固件升级</a></p>
</body>
</html>)rawliteral";
void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

String processor(const String& var){
  //Serial.println(var);
  if(var == "VController"){
    return VController;
  }else if(var == "VTool"){
    return VTool;
  }else if(var == "AirMax"){
    return UserVariables[0];
  }else if(var == "AirMin"){
    return UserVariables[1];
  }else if(var == "AirTimeOut"){
    return UserVariables[2];
  }else if(var == "ClawMax"){
    return UserVariables[3];
  }else if(var == "ClawMin"){
    return UserVariables[4];
  }else if(var == "ClawTimeOut"){
    return UserVariables[5];
  }
  return String();
  // else if(var == "Name"){
  //   return ToolName;
  // }
}

void setup() {
  Serial.begin(urat_baud);
  Serial485.begin(modbus_baud);
  IO_init();
  Mode_Key();
  Eeprom_init();
  pack_urat(1,1);//开机查询设备 
  while(TimeOut_Run != 4){
    check_urat();
  }
}

void loop() {
  if(Mode == "RUN")
  {
    Mode_run();  
  }
  
}
/* 功能 码 内容   
查询设备  01 FF FF  
控制设备  02 AA BB AA 自定 BB 动作位置
*/
void pack_urat(int function, int content) //功能码 内容 
{
  unsigned int crcData;
  if(function == 1){
    uratsend[0] = 0x01;
    uratsend[1] = 0xFF;
    uratsend[2] = 0xFF;
    crcData=CRC16_Check(uratsend,3);//crc16校验
	  uratsend[4] = crcData & 0xff;  //发送crc16校验低八位校验码
	  uratsend[3] = crcData >> 8;  //发送crc16校验高八位校验码
    serial2_write(uratsend,5); 
    TimeOut_Run = 1;
  }else if(function == 2)
  {
    uratsend[0] = 0x02;
    uratsend[1] = content >> 8;
    uratsend[2] = content & 0xff;
    crcData=CRC16_Check(uratsend,3);//crc16校验
	  uratsend[4] = crcData & 0xff;  //发送crc16校验低八位校验码
	  uratsend[3] = crcData >> 8;  //发送crc16校验高八位校验码
    serial2_write(uratsend,5); 
    TimeOut_Run = 2;
  }
}
// 功能码 01 
void check_urat(void) //00：功能码 01 02 ：内容 03 04:CRC 
{
	unsigned int crcData,tempData,temp;
  Serial.readBytes(receBuf, receCount);// 将接收到的信息使用readBytes读取  
  if (Serial.available() >= 5){ 
    receCount = Serial.available(); 
    for(uint8_t i=0; i<receCount; i++)
    {
     uratBuf[i] = Serial.read();    
    }  
    Serial.flush();   
		crcData = CRC16_Check(uratBuf,3);                     //核对校验码
    temp = (uratBuf[3]<<8)+uratBuf[4];
    

    if(temp == crcData)
    {
      TimeOut_Run = 4;
      count = 0;
      	switch(uratBuf[0])					//读取功能码
        {
        	case 1:  Get_name();							break;	//01 码 初始读下端信息	
        	default:break;												
      	}
    }
    receCount=0;
  }
}
void Get_name(){
   //ToolName = "Air";//控制器版本 // Air Null Claw
  if(uratBuf[1] == 1)
    ToolName = "Air";
  else if(uratBuf[1] == 2)
    ToolName = "Claw";
}
void time_ouT(){
    if(TimeOut_Run == 1){
      count ++;
      if(count == RequestTimeout){
        count = 0;
        TimeOut_Run = 3;
      }
    }else if(TimeOut_Run == 2){
      count ++;
    }else if(TimeOut_Run == 3){
      Serial.println(ToolName);
      TimeOut_Run = 4;
    }
}

void Eeprom_init() //开机读取
{
  //0 Air_Max 1 Air_Min 2 Air_TimeOut 3 Claw_Max 4 Claw_Min 5 Claw_TimeOut 
  EEPROM.begin(10);//EEProm init
  for(int j = 0 ;j < 6;j++){
    UserVariables[j] =  String(EEPROM.read(j));
  }
}
void Mode_run()
{
  check_modbus();
}
void IO_init()
{
  pinMode(LED_R,OUTPUT);  
  pinMode(LED_G,OUTPUT);   
  pinMode(LED_B,OUTPUT);
  pinMode(EN485,OUTPUT); 
  pinMode(Mode_key,INPUT_PULLUP);   
  digitalWrite(LED_R,LOW);  
  digitalWrite(LED_G,LOW);  
  digitalWrite(LED_B,LOW);
  ticker.attach(0.5, LED_Signal);
  TIME_OUT.attach(0.01, time_ouT);
}
void Mode_Key()
{
  delay(1000);
  if(digitalRead(Mode_key) == LOW)
     Mode = "SET";
  else if(digitalRead(Mode_key) == HIGH)
    Mode = "SET";
  if(Mode == "SET")
  {
    digitalWrite(LED_B,HIGH);
    Serial.println("SET"); 
    web_init();
  }else if(Mode == "RUN")
  {
    digitalWrite(LED_B,LOW);
    Serial.println("run");    
  }
}
void LED_Signal(){
  //count++;
  if(Mode == "SET")
  {
    if (WiFi.softAPgetStationNum() < 1)
     digitalWrite(LED_G,!digitalRead(LED_G));  
    else 
     digitalWrite(LED_G,HIGH);
    if (ToolName == "Null")
     digitalWrite(LED_R,HIGH);  
    else 
     digitalWrite(LED_R,LOW);
  }else if(Mode == "RUN")
  {
    if (ToolName == "Null"){
      digitalWrite(LED_R,HIGH);
      digitalWrite(LED_G,LOW);
    }
    else{
      digitalWrite(LED_R,LOW);
      digitalWrite(LED_G,HIGH);
    } 
  }
}
// WEB 部分
void web_init()
{
  wificonfig_wifiOff();
    boolean result = WiFi.softAP("Fox_V1.0", "Fox123456"); // "" => no password//a设置AP模式
    Serial.println(result == true ? "AP setup OK" : "AP setup failed");

    IPAddress myIP = WiFi.softAPIP();   
    Serial.print("Access Point IP address: ");Serial.println(myIP); //打印IP
    if (mdns.begin("espotaserver", myIP)) {
      Serial.println("MDNS responder started");
      }

    web_get();
    AsyncElegantOTA.begin(&server);    // Start AsyncElegantOTA
    server.begin();
    Serial.println("HTTP server started");
}
void web_get()
{
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    String inputMessage;
    String inputParam;
    char Wds = 0;
    if (request->hasParam(PARAM_INPUT_1)) {
      inputMessage = request->getParam(PARAM_INPUT_1)->value();
      inputParam = PARAM_INPUT_1;
      if (ToolName == "Air"){
        UserVariables[0] = String(inputMessage);
        EEPROM.write(0, inputMessage.toInt());
      }
        
      else if (ToolName == "Claw"){
        UserVariables[3] = String(inputMessage);
        EEPROM.write(3, inputMessage.toInt());
      }
        
    }
    else if (request->hasParam(PARAM_INPUT_2)) {
      inputMessage = request->getParam(PARAM_INPUT_2)->value();
      inputParam = PARAM_INPUT_2;
      if (ToolName == "Air"){
        UserVariables[1] = String(inputMessage);
        EEPROM.write(1, inputMessage.toInt());
      }
        
      else if (ToolName == "Claw"){
        UserVariables[4] = String(inputMessage);
        EEPROM.write(4, inputMessage.toInt());
      }
        
    }
    else if (request->hasParam(PARAM_INPUT_3)) {
      inputMessage = request->getParam(PARAM_INPUT_3)->value();
      inputParam = PARAM_INPUT_3;
      if (ToolName == "Air"){
        UserVariables[2] = String(inputMessage);
        EEPROM.write(2, inputMessage.toInt());
      }
        
      else if (ToolName == "Claw"){
        UserVariables[5] = String(inputMessage);
        EEPROM.write(5, inputMessage.toInt());
      }
        
    }
    else {
      inputMessage = "No message sent";
      inputParam = "none";
    }
    Serial.println(ToolName + inputMessage);
    EEPROM.commit();
    if (ToolName == "Null") //Air Null Claw
      request->send_P(200, "text/html", Null_TOOL, processor);
    else if (ToolName == "Air")
      request->send_P(200, "text/html", AIR_TOOL, processor);
    else if (ToolName == "Claw")
      request->send_P(200, "text/html", Claw_TOOL, processor);
  });
  server.on("/", HTTP_POST, [](AsyncWebServerRequest *request) {
    int params = request->params();
    for(int i=0;i<params;i++){
      AsyncWebParameter* p = request->getParam(i);
      if(p->isPost()){
        if (p->name() == PARAM_INPUT_4) {
          direction = p->value().c_str();
          Serial.print("Direction set to: ");
          Serial.println(direction);
        }
      }
    }
    if (ToolName == "Null") //Air Null Claw
      request->send_P(200, "text/html", Null_TOOL, processor);
    else if (ToolName == "Air")
      request->send_P(200, "text/html", AIR_TOOL, processor);
    else if (ToolName == "Claw")
      request->send_P(200, "text/html", Claw_TOOL, processor);

  });
}
void serial_write( uint8_t* send, uint8_t ssz)
{
  delayMicroseconds(50);

  for(uint8_t i=0; i<ssz; i++)
  {
    Serial485.write(send[i]);
    Serial485.flush(); 
  }
  //Serial.readBytes(recv, rsz);
}
void serial2_write( uint8_t* send, uint8_t ssz)
{
  delayMicroseconds(50);

  for(uint8_t i=0; i<ssz; i++)
  {
    Serial.write(send[i]);
    Serial.flush(); 
  }
  //Serial.readBytes(recv, rsz);
}
/****************************************************
/////**************CRC校验码生成函数 ***************/
unsigned int CRC16_Check(unsigned char *puchMsg,unsigned int usDataLen)//CRC16校验
{
  unsigned char uchCRCHi = 0xFF ; /* 高CRC字节初始化 */ 
  unsigned char uchCRCLo = 0xFF ; /* 低CRC 字节初始化 */ 
  unsigned int uIndex ; /* CRC循环中的索引 */ 
  while (usDataLen--) /* 传输消息缓冲区 */ 
  { 
    uIndex = uchCRCHi ^ *puchMsg++ ; /* 计算CRC */ 
    uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex] ; 
    uchCRCLo = auchCRCLo[uIndex] ; 
  } 
  return (uchCRCLo << 8 | uchCRCHi) ;  
}
/************************************************
//fuction:01 读单个或多个线圈状态
//主机发送 地址 + 功能码 + 起始地址（2个字节） + 线圈数量 （2个字节） + 校验码       
//从机返回 地址 + 功能码 + 字节数+ 线圈状态 + 校验码
**************************************************/
void readCoils(void)		//fuction:01 读单个或多个线圈状态
{
	unsigned int addr;
	unsigned int tempAddr;
	unsigned int byteCount;
	unsigned int bitCount;
	unsigned int crcData;
	unsigned int i,k;
	unsigned int tempData;
	unsigned char exit = 0;
	addr=(receBuf[2]<<8+receBuf[3]);
	tempAddr = addr;											//读取地址
	bitCount = (receBuf[4]<<8) + receBuf[5]; //读取的位个数
	byteCount = bitCount / 8;    //字节个数
	if(bitCount%8 != 0)							//不能整除加一个字节
		byteCount++; 
	for(k=0;k<byteCount;k++)
	{
		sendBuf[k+3]=0;
		for(i=0;i<8;i++)
		{
			getCoilVal(tempAddr,&tempData);//一位一位读取
			sendBuf[k+3] |= tempData<<i;//移位完成一个字节
			tempAddr++;//地址加1
			if(tempAddr>=addr+bitCount)//判读是否要读取的位都读好，读好后设定标志位
			{
				exit = 1; break; //跳出i语句循环
			}
		}
		if(exit == 1) break ;//跳出k语句循环
	}	
	sendBuf[0]=LocalAddr; 			//单片机控制板的地址
	sendBuf[1]=0x01;						//发送功能码
	sendBuf[2]=byteCount;				//发送字节数
	byteCount+=3;								//CRC16校验个数
	crcData=CRC16_Check(sendBuf,byteCount);//crc16校验
	sendBuf[byteCount++] = crcData & 0xff;  //发送crc16校验低八位校验码
	sendBuf[byteCount++] = crcData >> 8;  //发送crc16校验高八位校验码
	serial_write(sendBuf,byteCount); 
}
/************************************************
//fuction:02 读单个或多个线圈状态
//主机发送 地址 + 功能码 + 起始地址（2个字节） + 输入点数量 （2个字节） + 校验码       
//从机返回 地址 + 功能码 + 字节数+ 输入点状态 + 校验码
**************************************************/
void readInPutCoils(void)		//fuction:02 读取线圈输入（只读寄存器）状态
{
	unsigned int addr;
	unsigned int tempAddr;
	unsigned int byteCount;
	unsigned int bitCount;
	unsigned int crcData;
	unsigned int i,k;
	unsigned int tempData;
	unsigned char exit = 0;
	addr=(receBuf[2]<<8+receBuf[3])+10000;
	tempAddr = addr;											//读取地址
	bitCount = (receBuf[4]<<8) + receBuf[5]; //读取的位个数
	byteCount = bitCount / 8;    //字节个数
	if(bitCount%8 != 0)
		byteCount++; 
	for(k=0;k<byteCount;k++)
	{
		sendBuf[k+3]=0;
		for(i=0;i<8;i++)
		{
			getCoilVal(tempAddr,&tempData);//一位一位读取
			sendBuf[k+3] |= tempData<<i;//移位完成一个字节
			tempAddr++;//地址加1
			if(tempAddr>=addr+bitCount)//判读是否要读取的位都读好，读好后设定标志位
			{
				exit = 1; break; //跳出i语句循环
			}
		}
		if(exit == 1) break ;//跳出k语句循环
	}	
	sendBuf[0]=LocalAddr; 			//单片机控制板的地址
	sendBuf[1]=0x02;						//发送功能码
	sendBuf[2]=byteCount;				//发送字节数
	byteCount+=3;								//CRC16校验个数 //加上前面的地址，功能码，地址 共3+byteCount个字节
	crcData=CRC16_Check(sendBuf,byteCount);//crc16校验
	sendBuf[byteCount++] = crcData & 0xff;  //发送crc16校验低八位校验码
	sendBuf[byteCount++] = crcData >> 8;  //发送crc16校验高八位校验码
	serial_write(sendBuf,byteCount); 
}
/********function code : 03，读取多个寄存器值 ********/
////主机发送 地址 + 功能码 + 起始地址（2个字节） + 寄存器数量 （2个字节） + 校验码 
/////从机返回 地址 + 功能码 + 字节数+ 寄存器值 （N*2,一个数据2个字节） + 校验码
/*********************************************************/
void readRegisters(void)   //function code : 03，读取多个寄存器值
{
	unsigned int addr;
	unsigned int tempAddr;
	unsigned int crcData;
	unsigned int readCount;
	unsigned int byteCount;
	unsigned int i;
	unsigned int tempData = 0;//寄存器?? 
	addr = ((receBuf[2]<<8)+receBuf[3]);//+40000; //读取初始地址 , //+40000,保持寄存器偏移地址
	tempAddr = addr;
	readCount = (receBuf[4]<<8) + receBuf[5]; //要读的个数 ,整型
	byteCount = readCount * 2;                  //每个寄存器内容占高，低两个字节
	for(i=0;i<byteCount;i+=2,tempAddr++)
	{
		getRegisterVal(tempAddr,&tempData);    
		sendBuf[i+3] = tempData >> 8;        
		sendBuf[i+4] = tempData & 0xff;  
	}
	sendBuf[0]=LocalAddr; 			//单片机控制板的地址
	sendBuf[1]=0x03;						//发送功能码
	sendBuf[2]=byteCount;				//发送字节数
	byteCount+=3;								//CRC16校验个数
	crcData=CRC16_Check(sendBuf,byteCount);//crc16校验
	sendBuf[byteCount++] = crcData & 0xff;  //发送crc16校验低八位校验码
	sendBuf[byteCount++] = crcData >> 8;  //发送crc16校验高八位校验码
  serial_write(sendBuf,byteCount);  
}
/*************************************************************
//fuction:05 ,强制单个线圈
//////主机发送 地址 + 功能码 + 线圈地址（2个字节） + 写入值（2个字节）（置零：0x0000 ;;置一：0xff00） +校验码
//////从机返回 地址 + 功能码 + 线圈地址（2个字节） + 写入值（2个字节）（置零：0x0000 ;;置一：0xff00） +校验码
************************************************************/
void forceSingleCoil(void)  //fuction:05 ,强制单个线圈
{
	unsigned int addr;
	unsigned int tempAddr;
	unsigned int tempData = 0;
	unsigned int ONoff;
	unsigned char i;
	addr = (receBuf[2]<<8)+receBuf[3]; //读取初始地址 
	tempAddr = addr;//读取地址
	ONoff = (receBuf[4]<<8) + receBuf[5]; 
	if(ONoff==0xff00)	      tempData = 1;//设为ON
	else if(ONoff==0x0000)  tempData = 0;//设为OFF
	setCoilVal(tempAddr,tempData); 
	for(i=0;i<receCount;i++)
		sendBuf[i] = receBuf[i];
	serial_write(sendBuf,receCount);
}
/****************fuction:06设置单个寄存器**********************************************************/
//主机发送：从机地址 + 功能码 + 寄存器地址（2个字节，先寄存器高位，在寄存器低位）+数据写入值（2个字节，先高位再低位）+ CRC16校验（低位再高位）
//从机返回：从机地址 + 功能码 + 寄存器地址（2个字节，先寄存器高位，在寄存器低位）+数据写入值（2个字节，先高位再低位）+ CRC16校验（低位再高位）
/****************************************************************************************************/
void presetSingleRegister(void)  //fuction:06设置单个寄存器
{
	unsigned int addr;
	unsigned int tempAddr;
	unsigned int tempData;
	unsigned char i;
	addr = (receBuf[2]<<8)+receBuf[3]; //读取初始地址 
	tempAddr = addr;//+40000;//读取地址
	tempData = (receBuf[4]<<8) + receBuf[5];
	setRegisterVal(tempAddr,tempData);
	for(i=0;i<receCount;i++)
		sendBuf[i] = receBuf[i];
  serial_write(sendBuf,8); 
}
void getCoilVal(unsigned int addr,unsigned int *tempData)//取线圈状态
{
	switch(addr)
	{
    case 0x0000: *tempData = DO_00001;
		break;
    case 0x0001: *tempData = DO_00002;
		break;
    case 0x0002: *tempData = DO_00003;
		break;
    case 0x0003: *tempData = DO_00004;
		break;
    case 0x0004: *tempData = DO_00005;
		break;
    case 0x0005: *tempData = DO_00006;
		break;
    case 0x0006: *tempData = DO_00007;
		break;
    case 0x0007: *tempData = DO_00008;
		break;
    case 0x0008: *tempData = DO_00009;
		break;
    case 0x0009: *tempData = DO_00010;
		break;
		default:break;
	}
}
/*******************************读取寄存器内容函数************/
void getRegisterVal(unsigned int addr,unsigned int *tempData)  //读取寄存器内容函数*
{
	switch(addr)
	{
				case 0x0000: *tempData = Rs_0;
												break;
				case 0x0001: *tempData = Rs_1;
												break;
				case 0x0002: *tempData = Rs_2;
												break;
				case 0x0003: *tempData = Rs_3;
												break;
				case 0x0004: *tempData = Rs_4;
												break;
				case 0x0005: *tempData = Rs_5;
												break;
				case 0x0006: *tempData = Rs_6;
												break;
				case 0x0007: *tempData = Rs_7;
												break;
				case 0x0008: *tempData = Rs_8;
												break;
				case 0x0010: *tempData = R_0;
												break;
				case 0x0011: *tempData = R_1;
												break;
				case 0x0012: *tempData = R_2;
												break;
				case 0x0013: *tempData = R_3;
												break;
				case 0x0014: *tempData = R_4;
												break;
				case 0x0015: *tempData = R_5;
												break;	
				case 0x0016: *tempData = R_6;
												break;
				case 0x0017: *tempData = R_7;
												break;
				case 0x0018: *tempData = R_8;
												break;												
				default:
					break;
	}
}
void setCoilVal(unsigned int addr,unsigned int tempData)//设定单一线圈状态 
{
	switch(addr)
	{
		case 0x0000: DO_00001 = tempData;
		break;
		case 0x0001: DO_00002 = tempData;
		break;
		case 0x0002: DO_00003 = tempData;
		break;
		case 0x0003: DO_00004 = tempData;
		break;
		case 0x0004: DO_00005 = tempData;
		break;
		case 0x0005: DO_00006 = tempData;
		break;
		case 0x0006: DO_00007 = tempData;
		break;
		case 0x0007: DO_00008 = tempData;
		break;
		case 0x0008: DO_00009 = tempData;
		break;
		case 0x0009: DO_00010 = tempData;
		break;
		default:break;
	}
}
void setRegisterVal(unsigned int addr,unsigned int tempData) //设置寄存器内容函数
{
	//cmd[addr-40000] = tempData;
	switch(addr)
	{
				case 0x0000: Rs_0 = tempData;
												break;
				case 0x0001: Rs_1 = tempData;
												break;
				case 0x0002: Rs_2 = tempData;
												break;
				case 0x0003: Rs_3 = tempData;
												break;
				case 0x0004: Rs_4 = tempData;
												break;
				case 0x0005: Rs_5 = tempData;
												break;
				case 0x0006: Rs_6 = tempData;
												break;
				case 0x0007: Rs_7 = tempData;
												break;
				case 0x0008: Rs_8 = tempData;
												break;
				case 0x0010: S_0 = tempData;
												break;
				case 0x0011: S_1 = tempData;
												break;
				case 0x0012: S_2 = tempData;
												break;
				case 0x0013: S_3 = tempData;
												break;
				case 0x0014: S_4 = tempData;
												break;
				case 0x0015: S_5 = tempData;
												break;
				case 0x0016: S_6 = tempData;
												break;		
				case 0x0017: S_7 = tempData;
												break;
				case 0x0018: S_8 = tempData;
												break;												
				default:
					break;
	}
} 
void check_modbus(void)
{
	unsigned int crcData,tempData,temp;
  Serial485.readBytes(receBuf, receCount);// 将接收到的信息使用readBytes读取  
  if (Serial485.available() >= 8){ 
    receCount = Serial485.available();  
    for(uint8_t i=0; i<receCount; i++)
    {
     receBuf[i] = Serial485.read();    
    }  
    Serial485.flush();   
		if(receBuf[0]==LocalAddr)		//核对地址
		{
      if(receBuf[1]<10)
			{          
         	if(receCount>=7)
        	{
              crcData = CRC16_Check(receBuf,6);                     //核对校验码
          		temp = (receBuf[7]<<8)+receBuf[6];
          		if(temp == crcData)
          		{
            		switch(receBuf[1])					//读取功能码
            		{
              			case 1:  readCoils();							break;	//读取线圈输出状态(一个或多个) 	
              			case 2:  readInPutCoils();				break;	//读取线圈输入（只读寄存器）状态
              			case 3:	 readRegisters();	 				break;  //读取多个寄存器值
              			case 5:	 forceSingleCoil(); 			break;	//强制单个线圈
              			case 6:	 presetSingleRegister();  break;  //设置单个寄存器
              			default:break;												
            		}
          		}
	        }
	    } 
     	else if(receBuf[1]==16)
     	{
        Serial485.write("16M");	
     	}
      receCount=0;	
		}
  }
}