#define LocalAddr 0x06				//Modbus地址

static uint8_t Tool[8]={0x01 , 0x02 , 0x03 , 0x04 , 0x05 , 0x06 , 0x07 , 0x08};
unsigned char sendBuf[50];
unsigned char receBuf[50];
unsigned char receCount; //接收数据计数
int count;    // 计数用变量

int LED_G = 18;
int LED_R = 19;
int M_EN = 4;
int M_PWM = 5;

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


void setup() {
  // put your setup code here, to run once:
  Serial2.begin(115200);
  Serial.begin(9600); //设置串口波特率9600
  pinMode(LED_G,OUTPUT);
  pinMode(LED_R,OUTPUT);
  pinMode(M_EN,OUTPUT);
  pinMode(M_PWM,OUTPUT);
  digitalWrite(M_EN,LOW);
  digitalWrite(M_PWM,LOW);
  digitalWrite(LED_G,LOW);
  digitalWrite(LED_R,LOW);
}

void loop() {
  check_modbus();
}

//串口发送指定数据，然后接收数据
void serial_write( uint8_t* send, uint8_t ssz)
{
  delayMicroseconds(50);
  for(uint8_t i=0; i<ssz; i++)
  {
    Serial2.write(send[i]);
    Serial2.flush(); 
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
/****************fuction:06设置单个寄存器ok**********************************************************/
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
  Serial2.readBytes(receBuf, receCount);// 将接收到的信息使用readBytes读取  
  //Serial.print("Hi ");
  //Serial2.write("16M");	
  if (Serial2.available() >= 8){ 
    receCount = Serial2.available();  
    for(uint8_t i=0; i<receCount; i++)
    {
     receBuf[i] = Serial2.read();    
    }  
    Serial2.flush();   
		if(receBuf[0]==LocalAddr)		//核对地址
		{
      //Serial.print("OK");
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
        Serial2.write("16M");	
     	}
      receCount=0;	
		}
  }
}
