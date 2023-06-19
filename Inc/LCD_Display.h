
// 			 LCD, SPI Functions
void     Configure_LCD_Pins(void);
void     Configure_SPI1(void);
void     Activate_SPI1(void);
void     Send_SPI_Byte(uint8_t);
void     Initialise_LCD_Controller(void);
void     Reset_Display_Pin(uint8_t);
void     Chip_Select_Pin(uint8_t);
void     InstructionOrData(char);
void     Write_Data_To_LCD(unsigned char data);
void     Write_Command_To_LCD(unsigned char command);
void     Copy_Data_Buffer_To_LCD(void);
void     Fill_Page(unsigned char);
void     Write_Test_Character(unsigned char l0, unsigned char l1,unsigned char l2,unsigned char l3,unsigned char l4,unsigned char l5,unsigned char l6,unsigned char l7);
void     pixel(int x,int y, int colour);
void     Set_Contrast(unsigned int o);
void 		 Display_Character_A(void);
void     Display_Hello(void);
void     Clear_Screen(void);
void     locate(int x, int y);
int      columns(void);
int      rows(void);
int      width(void);
int      height(void);
void     set_font(unsigned char* f);
void     character(int x, int y, int c);
int      put_char(int);
int      put_string(int x, int y, char* stringToSend);
