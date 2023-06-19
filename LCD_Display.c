#include "main.h"
#include "LCD_Display.h"
#include <stdio.h>
#include <string.h>

unsigned int char_x, char_y, orientation;
unsigned char* font;
unsigned char buffer[512];    // Display data buffer for LCD

void LCD_Display_Config(void) {
/* Configure Display */
	Configure_LCD_Pins();
	Configure_SPI1();
	Activate_SPI1();
	Clear_Screen();
	Initialise_LCD_Controller();
}

int put_string(int x, int y, char* stringToSend){
	locate(x,y);
	int width_of_font = font[1]-1;
	int length = strlen(stringToSend);
	for(int i=0; i<length;i++){
		put_char(stringToSend[i]);
		x+=width_of_font; 
		locate(x,y);
	}
	return 1;
}


int put_char(int value)
{
    if (value == '\n') {    // new line
        char_x = 0;
        char_y = char_y + font[2];
        if (char_y >= height() - font[2]) {
            char_y = 0;
        }
    } else {
        character(char_x, char_y, value);
        Copy_Data_Buffer_To_LCD();
    }
    return value;
}
 
void character(int x, int y, int c)
{
    unsigned int hor,vert,offset,bpl,j,i,b;
    unsigned char* symbol;
    unsigned char z,w;
 
    if ((c < 31) || (c > 127)) return;   // test char range
 
    // read font parameter from start of array
    offset = font[0];                       // bytes / char
    hor    = font[1];                       // get hor size of font
    vert   = font[2];                       // get vert size of font
    bpl    = font[3];                       // bytes per line
 
    if (char_x + hor > width()) {
        char_x = 0;
        char_y = char_y + vert;
        if (char_y >= height() - font[2]) {
            char_y = 0;
        }
    }
 
    symbol = &font[((c -32) * offset) + 4]; // start of char bitmap
    w = symbol[0];                          // width of actual char
                                            // construct the char into the buffer
    for (j=0; j<vert; j++) {                //  vert line
        for (i=0; i<hor; i++) {             //  horz line
            z =  symbol[bpl * i + ((j & 0xF8) >> 3)+1];
            b = 1 << (j & 0x07);
            if (( z & b ) == 0x00) {
                pixel(x+i,y+j,0);
            } else {
                pixel(x+i,y+j,1);
            }
 
        }
    }
 
    char_x += w;
}
 


void set_font(unsigned char* f)
{
    font = f;
}

int width(void)
{
    if (orientation == 0 || orientation == 2) return 32;
    else return 128;
}
 
int height(void)
{
    if (orientation == 0 || orientation == 2) return 128;
    else return 32;
}

int columns(void)
{
    return width() / font[1];
}
 
 
 
int rows(void)
{
    return height() / font[2];
}

void locate(int x, int y)
{
    char_x = x;
    char_y = y;
}
 

void Clear_Screen(void){
	  memset(buffer,0x00,512);  // clear display buffer
	  Copy_Data_Buffer_To_LCD();
}

void Write_Test_Character(unsigned char l0, unsigned char l1,unsigned char l2,unsigned char l3,unsigned char l4,unsigned char l5,unsigned char l6,unsigned char l7){
		memset(buffer,0x00,512);  // clear display buffer
		buffer[0] = l0;
	  buffer[1] = l1;
	  buffer[2] = l2;
	  buffer[3] = l3;
	  buffer[4] = l4;
		buffer[5] = l5;
	  buffer[6] = l6;
	  buffer[7] = l7;
		Copy_Data_Buffer_To_LCD();
}
 
void pixel(int x, int y, int color)
{
    if(x > 128 || y > 32 || x < 0 || y < 0) return;
    if(color == 0)
      buffer[x + ((y/8) * 128)] &= ~(1 << (y%8));  // erase pixel
    else
      buffer[x + ((y/8) * 128)] |= (1  << (y%8));   // set pixel
}

void Set_Contrast(unsigned int o)
{
    Write_Command_To_LCD(0x81);      
    Write_Command_To_LCD(o & 0x3F);
}

void Display_Character_A(void)
{
	  memset(buffer,0x00,512);  // clear display buffer
		buffer[0] = 0x00;
	  buffer[1] = 0x7c;
	  buffer[2] = 0x7e;
	  buffer[3] = 0x13;
	  buffer[4] = 0x13;
		buffer[5] = 0x7e;
	  buffer[6] = 0x7c;
	  buffer[7] = 0x00;
		Copy_Data_Buffer_To_LCD();
}

void Display_Hello(void)
{
	// H = 0x0, 0x7f, 0x7f, 0x8, 0x8, 0x7f, 0x7f, 0x0
		memset(buffer,0x00,512);  // clear display buffer
		buffer[0] = 0x00;
	  buffer[1] = 0x7f;
	  buffer[2] = 0x7f;
	  buffer[3] = 0x08;
	  buffer[4] = 0x08;
		buffer[5] = 0x7f;
	  buffer[6] = 0x7f;
	  buffer[7] = 0x00;
	 // e = 0x0, 0x38, 0x7c, 0x54, 0x54, 0x5c, 0x18, 0x0
		buffer[8]  = 0x00;
	  buffer[9]  = 0x38;
	  buffer[10] = 0x7c;
	  buffer[11] = 0x54;
	  buffer[12] = 0x54;
		buffer[13] = 0x5c;
	  buffer[14] = 0x18;
	  buffer[15] = 0x00;
		 // l = 0x0, 0x0, 0x41, 0x7f, 0x7f, 0x40, 0x0, 0x0
		buffer[16]  = 0x00;
	  buffer[17]  = 0x00;
	  buffer[18] = 0x41;
	  buffer[19] = 0x7f;
	  buffer[20] = 0x7f;
		buffer[21] = 0x40;
	  buffer[22] = 0x00;
	  buffer[23] = 0x00;
		// l = 0x0, 0x0, 0x41, 0x7f, 0x7f, 0x40, 0x0, 0x0
		buffer[24]  = 0x00;
	  buffer[25]  = 0x00;
	  buffer[26] = 0x41;
	  buffer[27] = 0x7f;
	  buffer[28] = 0x7f;
		buffer[29] = 0x40;
	  buffer[30] = 0x00;
	  buffer[31] = 0x00;
		// o = 0x0, 0x38, 0x7c, 0x44, 0x44, 0x7c, 0x38, 0x0
		buffer[32] = 0x00;
	  buffer[33] = 0x38;
	  buffer[34] = 0x7c;
	  buffer[35] = 0x44;
	  buffer[36] = 0x44;
		buffer[37] = 0x7c;
	  buffer[38] = 0x38;
	  buffer[39] = 0x00;
		
		Copy_Data_Buffer_To_LCD();
}


void Fill_Page(unsigned char Page){
	if(Page == 0){  //Line 1
		for(int i=0;i<128;i++){
			buffer[i] = 0xff;
		}
	}
	if(Page == 1){  //Line 2
		for(int i=128;i<256;i++){
			buffer[i] = 0xff;
		}
	}
	if(Page == 2){  //Line 3
		for(int i=256;i<384;i++){
			buffer[i] = 0xff;
		}
	}
	if(Page == 3){  //Line 4
		for(int i=384;i<512;i++){
			buffer[i] = 0xff;
		}
	}
	Copy_Data_Buffer_To_LCD();
}


void Copy_Data_Buffer_To_LCD(void)
{    
    //To set 8-bit column address data will start, the address is split into 2 nibbles
	  //to send the lower nibble, 0x0 then the lower nibble of the 8 bit address, so nibble of 0 would be 0x00
	  //to send the higher nibble 0x1 then the higher nibble of the 8 bit address, so nibble of 0 would be 0x10
	  //to set the column start to 0, send 0x00 then 0x10
	
	  int i=0;
	  
	  // Page 0
    Write_Command_To_LCD(0x00);      // set column low nibble to 0  
    Write_Command_To_LCD(0x10);      // set column hi nibble to 0
    Write_Command_To_LCD(0xB0);      // set page address  0
    InstructionOrData('I');
    for(i=0; i<128; i++) {
        Write_Data_To_LCD(buffer[i]);
    }
 
    // Page 1
    Write_Command_To_LCD(0x00);      // set column low nibble to 0
    Write_Command_To_LCD(0x10);      // set column hi nibble to 0
    Write_Command_To_LCD(0xB1);      // set page address  1
    InstructionOrData('I');
    for(i=128; i<256; i++) {
        Write_Data_To_LCD(buffer[i]);
    }

    //Page 2
    Write_Command_To_LCD(0x00);      // set column low nibble to 0
    Write_Command_To_LCD(0x10);      // set column hi nibble to 0
    Write_Command_To_LCD(0xB2);      // set page address  2
    InstructionOrData('I');
    for(i=256; i<384; i++) {
        Write_Data_To_LCD(buffer[i]);
    }
 
    //Page 3
    Write_Command_To_LCD(0x00);      // set column low nibble to 0
    Write_Command_To_LCD(0x10);      // set column hi nibble to 0
    Write_Command_To_LCD(0xB3);      // set page address  3
    InstructionOrData('I');
 
    Chip_Select_Pin(1);
    
    for(i=384; i<512; i++) {
        Write_Data_To_LCD(buffer[i]);
    }
}

void Initialise_LCD_Controller(void){
	InstructionOrData('I');
	Reset_Display_Pin(0);
	LL_mDelay(1);
	Reset_Display_Pin(1);
	LL_mDelay(5);
  Write_Command_To_LCD(0xAE);   //  display off
  Write_Command_To_LCD(0xA2);   //  bias voltage
 
  Write_Command_To_LCD(0xA0);
  Write_Command_To_LCD(0xC8);   //  column normal
 
  Write_Command_To_LCD(0x22);   //  voltage resistor ratio
  Write_Command_To_LCD(0x2F);   //  power on

  Write_Command_To_LCD(0x40);   //  start line = 0
  Write_Command_To_LCD(0xAF);   //  display ON
 
  Write_Command_To_LCD(0x81);   //  set contrast
  Write_Command_To_LCD(0x17);   //  set contrast
 
  Write_Command_To_LCD(0xA6);   // display normal	

}

void Reset_Display_Pin(uint8_t pinstate){
	if(pinstate == 0){
		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6);
	}
	else{
		LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6);
	}
}

void Chip_Select_Pin(uint8_t pinstate){
	if(pinstate == 0){
		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6);
	}
	else{
		LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_6);
	}
}

void InstructionOrData(char iod){
	if((iod == 'i') || (iod == 'I')){
		//Instruction Command
		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_8);
	}
	if((iod == 'd') || (iod == 'D')){
		LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_8);
	}
}

void Write_Data_To_LCD(unsigned char data)
{
    InstructionOrData('D');
    Chip_Select_Pin(0);
	  Send_SPI_Byte(data);
    Chip_Select_Pin(1);
}

void Write_Command_To_LCD(unsigned char command)
{
    InstructionOrData('I');
    Chip_Select_Pin(0);
	  Send_SPI_Byte(command);
    Chip_Select_Pin(1);
}

void Send_SPI_Byte(uint8_t byte){
		//LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6);
	  //while(LL_SPI_IsActiveFlag_BSY(SPI1))
		LL_SPI_TransmitData8(SPI1, (unsigned char)byte);
		while(LL_SPI_IsActiveFlag_BSY(SPI1));
	  //LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_6);
}

void Configure_LCD_Pins(void){
	/* Configure Instruction_Or_Data Pin connected to GPIOA8 (D7) */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_GPIO_SetPinMode(  GPIOA, LL_GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinSpeed( GPIOA, LL_GPIO_PIN_8, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(  GPIOA, LL_GPIO_PIN_8, LL_GPIO_PULL_UP);
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_8);
	
	/* Configure LCD Reset Pin connected to GPIOA6 (D12) */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_GPIO_SetPinMode(  GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinSpeed( GPIOA, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(  GPIOA, LL_GPIO_PIN_6, LL_GPIO_PULL_UP);
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6);
	
	/* Configure LCD Chip Select Pin connected to GPIOB6 (D10) */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  LL_GPIO_SetPinMode(  GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinSpeed( GPIOB, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(  GPIOB, LL_GPIO_PIN_6, LL_GPIO_PULL_UP);
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_6);
	
	/* Configure SCK Pin connected to GPIOA5 (D13) */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_GPIO_SetPinMode(  GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_5, LL_GPIO_AF_5);
  LL_GPIO_SetPinSpeed( GPIOA, LL_GPIO_PIN_5, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(  GPIOA, LL_GPIO_PIN_5, LL_GPIO_PULL_DOWN);

  /* Configure MOSI Pin connected to GPIOA7 (D11) */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_GPIO_SetPinMode(  GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_7, LL_GPIO_AF_5);
  LL_GPIO_SetPinSpeed( GPIOA, LL_GPIO_PIN_7, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(  GPIOA, LL_GPIO_PIN_7, LL_GPIO_PULL_DOWN);
	
}

void Configure_SPI1(void){


	//NVIC_SetPriority(SPI1_IRQn, 0);
  //NVIC_EnableIRQ(SPI1_IRQn);
	
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

  /* Configure SPI1 communication */
  LL_SPI_SetBaudRatePrescaler(SPI1, LL_SPI_BAUDRATEPRESCALER_DIV8);
  LL_SPI_SetTransferDirection(SPI1,LL_SPI_HALF_DUPLEX_TX);
  LL_SPI_SetClockPhase(SPI1, LL_SPI_PHASE_2EDGE);
  LL_SPI_SetClockPolarity(SPI1, LL_SPI_POLARITY_HIGH);
  
	/* Reset value is LL_SPI_MSB_FIRST */
  //LL_SPI_SetTransferBitOrder(SPI1, LL_SPI_MSB_FIRST);
  LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_8BIT);
  LL_SPI_SetNSSMode(SPI1, LL_SPI_NSS_SOFT); //Chip select handled by software
  LL_SPI_SetMode(SPI1, LL_SPI_MODE_MASTER);
	
	/* Configure SPI1 transfer interrupts */
  /* Enable TXE   Interrupt */
  //LL_SPI_EnableIT_TXE(SPI1);
  /* Enable SPI1 Error Interrupt */
  //LL_SPI_EnableIT_ERR(SPI1);
}

void Activate_SPI1(void){
	/* Enable SPI1 */
  LL_SPI_Enable(SPI1);
}
