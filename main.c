#include <REG_MPC82G516.h>
#include <string.h>
#include <intrins.h>

#define bh1750_addr   0x23
#define BH1750_CONT_HRES_MODE      0x10

typedef unsigned char  uint8_t;
typedef unsigned int   uint16_t;

sbit SERVO_PIN = P1^0; // 假設馬達接在 P2.0
volatile unsigned int reload_high = 65036; 
volatile unsigned int reload_low = 46036;  
volatile bit servo_pin_high = 0;

sbit SDA = P3^0;
sbit SCL = P3^1;


sbit SS = P1^4; //把P1.4設成SS   

volatile unsigned int timer0_ms = 0;
volatile bit read_bh1750_status = 0; 

static int current_angle = 90;

void delay(void){
  #pragma asm
 delay_120:
  MOV R5,#4
 aa:
  MOV R6,#4
 bb:
  MOV R7,#0FFH
 cc:											  
  DJNZ R7,cc
  DJNZ R6,bb
  DJNZ R5,aa
  RET
 #pragma endasm
}

static void t2_init_for_udelay(void)
{
    TR2 = 0;
    T2CON = 0x00;  // Fosc/12, auto-reload
    TF2 = 0;
}

void udelay(unsigned int us)
{
    // 12MHz / 12T 模式下，1個 _nop_() 大約是 1us
    // 扣除函數呼叫開銷，稍微調整迴圈次數
    while (us--) {
        _nop_(); 
        _nop_(); // 視實際硬體速度調整 nop 數量
    }
}

void mdelay(unsigned int ms)
{
    while (ms--) udelay(1000);
}

void timer0_init(void){
	TMOD = (TMOD & 0xF0) | 0x01; //保留timer1,清空timer0且設為mode1
	TH0 = 0xFC;  //把preload值設好	65535 - 我要計時的時間
	TL0 = 0x18;
	ET0 = 1; //開timer0中斷斷
	EA = 1;
	TR0 = 1;
}


void uart_init(){
	SCON = 0x50;
	AUXR2 |= 0x40;
	TMOD = (TMOD & 0x0F) | 0x20; 
	TH1 = 217;
	TL1 = 217;
	TR1 = 1;
	ES = 1;
	EA = 1;
}


void send_string(const char* string) { 
	int i;
	int size = strlen(string);
	for (i=0 ; i<size ; i++) {
		SBUF = string[i];
		delay();
	}		
}

// 將 1 byte 轉為 "01010101\r\n\0" 到 caller 提供的 buf（至少 11 bytes）
static void byte_to_bitstring_buf(unsigned char v, char *buf)
{
    unsigned char i;
    for (i = 0; i < 8; i++) {
        buf[i] = (v & (0x80 >> i)) ? '1' : '0';  // MSB→LSB
    }
    buf[8]  = '\r';
    buf[9]  = '\n';
    buf[10] = '\0';
}

static void i2c_init(void)
{
	// P3.1、P3.0 設為 Open-Drain（PxM1=1 且 PxM0=1）
    P3M1 |= (1 << 1) | (1 << 0);
    P3M0 |= (1 << 1) | (1 << 0);
	SDA = 1;
	SCL = 1;
}

static void i2c_start(void){   
	SDA = 1;
	SCL = 1;
	udelay(5);
	SDA = 0;
	udelay(5);
	SCL = 0;
}

static void i2c_stop(void){
    SDA = 0;
	SCL = 1;
	udelay(5);
	SDA = 1;
	udelay(5);
}

static void i2c_write_bit(unsigned char data_out){  
	SDA = data_out;
	udelay(5);
	SCL = 1;
	udelay(5); //SCL	調成高電位後要delay
	SCL = 0;    
}

static unsigned char i2c_read_bit(void){
    unsigned char data_in;
	SDA = 1;               // 釋放，讓從機驅動 SDA
	udelay(5);
	SCL = 1;
	udelay(5);
	data_in = SDA;
	SCL = 0;
	return data_in;
}

static int i2c_write_byte(uint8_t byte){
    int i;
	for (i=0 ; i<8 ; i++) {
		i2c_write_bit(byte & 0x80);
		byte <<= 1;
	}
	//如果收到ack	代表傳輸完成
	if (i2c_read_bit() == 0) {
		return 0;
	}
	//如果收到NACK代表失敗
	return 1;
}

static uint8_t i2c_read_byte(unsigned char ack){
     uint8_t data_in = 0;
	 int i;
	 for (i=0 ; i<8 ; i++) {	 	
		data_in <<= 1;
		data_in |= i2c_read_bit();
	 }
	 i2c_write_bit(ack? 0 : 1);
	 return data_in;

}

void prepare_servo_reload(unsigned char angle) {
    unsigned int high_us;
    if (angle > 180) angle = 180;
    
    // 你的線性換算公式
    high_us = 500 + (unsigned int)(((unsigned long)angle * 2000) / 180);
    
    ET2 = 0; // 關閉 Timer2 中斷保護 16-bit 寫入
    reload_high = 65536u - high_us;
    reload_low  = 65536u - (20000u - high_us);
    ET2 = 1;
}

static void read_bh1750(void){
	uint16_t read_data = 0;
    unsigned char msb;
    unsigned char lsb;
    char bits[11];                  // 本地固定陣列（不回傳它！）

    i2c_init();
    i2c_start();
    i2c_write_byte((bh1750_addr<<1) | 0);
    i2c_write_byte(BH1750_CONT_HRES_MODE);
    i2c_stop();
    mdelay(1000);

	//send_string("address check\r\n");

    i2c_start();
    i2c_write_byte((bh1750_addr<<1) | 1);
    read_data  = (uint16_t)i2c_read_byte(1);
    read_data <<= 8;
    read_data |= (uint16_t)i2c_read_byte(0);
    i2c_stop();
	mdelay(1000);

	//send_string("get data\r\n");

    send_string("bh1750 = \r\n");

    msb = (unsigned char)(read_data >> 8);
    byte_to_bitstring_buf(msb, bits);
    send_string(bits);              // ? 傳字串，不是傳單一位元組

    lsb = (unsigned char)(read_data & 0xFF);
    byte_to_bitstring_buf(lsb, bits);
    send_string(bits);

	if (read_data > 500) { 
        send_string("Threshold Exceeded! Servo -> 180\r\n");
        prepare_servo_reload(180);
    } else {
        send_string("Light Low. Servo -> 0\r\n");
        prepare_servo_reload(0);
    }

}

/*
void spi_init(void){
	SPCTL = 0x00;
	SPSTAT = 0xC0;
	SPCTL = 0xD1;
	SS = 1;		
}

void spi_start(void){
	SS = 0;
	delay(); 
}

void spi_close(void){
	SS = 1;
	delay();
}

void spi_transfer(const char* data){
	int i;
	int data_size = strlen(data) ;
	for (i=0 ; i<data_size ; i++) {
		SPSTAT = 0xC0;              // 送前先清 SPIF/WCOL
		SPDAT = data[i]	;
		while ((SPSTAT & 0x80) == 0); //等待	SPIF變為1，確認資料傳輸完成
		SPSTAT = 0x80; 
	}
}
 */


void main(void){
    uart_init();
    t2_init_for_udelay();
    timer0_init(); 
    
    // 初始化馬達位置
    prepare_servo_reload(0); 
    ET2 = 1; // 啟動馬達中斷
    EA = 1;
    TR2 = 1; // 啟動 Timer2

    send_string("System Initialized\r\n");

    while (1) {
        if (read_bh1750_status == 1) {
            read_bh1750_status = 0;
            read_bh1750(); // 讀取並判斷閥值
        }
    }
	
	/*
	P0 = 0xFF;
	while(1) {
		int time = 0;
		P0 &= 0x0F;
		while (time < 32000) {
			time++;
		}
		time = 0;
		P0 |=0xF0;
		P0 &= 0xF0;
		while (time < 32000) {
			time++;
		}
		P0 |= 0x0F;

	}
	*/
}

void uart() interrupt 4{
	if (RI) {                      // ★ 收到資料
        RI = 0;                    // ★ 清 RI
    }
    if (TI) {                      // ★ 傳送完成
        TI = 0;                    // ★ 清 TI
    }
}

void timer0_isr(void) interrupt 1
{
 	TF0 = 0;
	TH0 = 0xFC;  //把preload值設好	65535 - 我要計時的時間
	TL0 = 0x18;
	if (++timer0_ms >= 200) {
		timer0_ms = 0;
		read_bh1750_status = 1;
	}
}

void timer2_isr(void) interrupt 5 {
    TF2 = 0; 
    if (servo_pin_high) {
        SERVO_PIN = 0;
        servo_pin_high = 0;
        TH2 = (unsigned char)(reload_low >> 8);
        TL2 = (unsigned char)(reload_low & 0xFF);
    } else {
        SERVO_PIN = 1;
        servo_pin_high = 1;
        TH2 = (unsigned char)(reload_high >> 8);
        TL2 = (unsigned char)(reload_high & 0xFF);
    }
}