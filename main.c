/**
 * @file    main.c
 * @brief   This program simulates  the tilt register from the accelerometer, calculates/checks the IPv4 type checksum,
 *          creates a packet of data, stores and retrives the packet using the EEPROM memory, generates and handles
 *          simple GUI based on an LCD and joistick.
 * @author  Karol Wojslaw (10746230, karol.wojslaw@student.manchester.ac.uk)
 */

#include "main.h"

#include "Time_Delays.h"
#include "Clk_Config.h"
#include "LCD_Display.h"

#include "Arial_9.h"

#include <stdio.h>
#include <string.h>

#define ACCELADR 0x98   // I2C Address of the accelerometer
#define EEPROMADR 0xA0  // I2C Address of the EEPROM
#define PACKET_SIZE	68	// Size of the packet in bytes

/* GUI State type definition */
typedef enum {
    GUI_DEST = 0,         // Destination address
    GUI_SRC = 1,          // Source address
    GUI_LENGTH = 2,       // Packet length
    GUI_PAYLOAD = 3,      // Tilt register value
    GUI_CHECKSUM = 4,     // Checksum value
    GUI_CHECKSUM_TEST = 5,  // Checksum test
    GUI_SAMPLED = 6,      // Accelerometer register read
    GUI_WRITTEN = 7,      // Data written to the EEPROM
    GUI_RETRIVED = 8      // Data retrived from the EEPROM
} gui_state_type_t;

/* Packet structure type definition */
typedef struct {
  uint32_t dest;            // Destination address (4 bytes, disp in hex)
  uint32_t src;             // Source address (4 bytes, disp in hex)
  uint16_t length;          // Length of data in the payload (2 bytes, disp in hex)
  uint8_t payload[54];      // Data (54 bytes, disp in bin). First byte used to store a tilt reg value
  uint16_t checksum;        // IPv4 header type checksum (2 bytes, disp in hex)
  uint16_t recalc_checksum; // Auxiliary checksum element for recalculated val (for comparison only)
} packet_type_t;

/* Function declarations */
void system_config(void); // General
void gpio_config(void);   // GPIO
void i2c_1_config(void);  // I2C
void lcd_config(void);    // LCD
void gui_update(gui_state_type_t state, packet_type_t packet);  // GUI
uint16_t checksum_calculate(packet_type_t packet);  // Checksum
// Accelerometer:
void accelerometer_config(void);  
uint8_t accelerometer_read(void); 
// EEPROM:
void eeprom_write(packet_type_t *packet, uint8_t size);
void eeprom_read(packet_type_t* packet, uint8_t size);
void eeprom_ack_polling(void);
// Joystick:
void joystick_config(void);
uint8_t joystick_centre(void);
uint8_t joystick_up(void);
uint8_t joystick_down(void);
uint8_t joystick_left(void);
uint8_t joystick_right(void);

/* -------------------------------------------------- */
/* ---------------------- Main ---------------------- */

int main(void) {
	/* Configuration and initialisation */
	system_config();        // General
	lcd_config();           // LCD
	gpio_config();          // GPIO
	joystick_config();      // Joystick
	i2c_1_config();         // I2C
	accelerometer_config(); // Accelerometer

	packet_type_t packet = {  // Create a packet variable (based on structure type)
		.dest = 0xcccccccc,     // Destination address
		.src = 0xdddddddd,      // Source address
		.length = 54,           // Length of data in the payload
		.checksum = 0xaa74,     // IPv4 header type checksum
		.recalc_checksum = 0    // Checksum calculated after retriving the packet from memory (for comparison)
	};

	for (int i=0; i<packet.length; i++) { packet.payload[i] = 0; }  // Zero-initialise the payload field in the packet

	gui_state_type_t gui_state = GUI_PAYLOAD; // Display payload value by default
	gui_update(gui_state, packet); // Update the LCD

	while (1) { // Main Loop
	  if(joystick_centre()) {
		  packet.payload[0] = accelerometer_read();     // Read the tilt register and store the value in the payload field of the packet
			packet.checksum = checksum_calculate(packet); // Calculate and update the checksum

			gui_update(GUI_SAMPLED, packet);
			gui_update(GUI_PAYLOAD, packet);
			gui_state = GUI_PAYLOAD;

		} else if(joystick_right()) {
			eeprom_write(&packet, PACKET_SIZE); // Write 68-byte packet to the EEPROM

			gui_update(GUI_WRITTEN, packet);
			gui_update(GUI_PAYLOAD, packet);
			gui_state = GUI_PAYLOAD;
      
		} else if(joystick_left()) {
			eeprom_read(&packet, PACKET_SIZE);  // Retrive the packet from EEPROM

			gui_update(GUI_RETRIVED, packet);	
			gui_update(GUI_PAYLOAD, packet);	
			gui_state = GUI_PAYLOAD;
      
		} else if(joystick_up()) {
			if(gui_state > 0) {gui_state--;}  // Move one state up (display the next field of the packet)
			if(gui_state == GUI_CHECKSUM_TEST) {packet.recalc_checksum = checksum_calculate(packet);} // Recalculate checksum before testing it
			gui_update(gui_state, packet);

		} else if(joystick_down()) { 
			if(gui_state < 5) {gui_state++;}  // Move one state down (display the previous field of the packet)
			if(gui_state == GUI_CHECKSUM_TEST) {packet.recalc_checksum = checksum_calculate(packet);} // Recalculate checksum before testing it
			gui_update(gui_state, packet);

		}
  }
}

/* -------------------------------------------------- */
/* --------------------- General -------------------- */

/**
 * @brief Configure system clock and SysTick timer
 */
void system_config(void) {
  SystemClock_Config();    // Configure the system clock to 84.0 MHz
  SysTick_Config_MCE2(us); // Configure time unit and initialise SysTick timer for time measurement
}

/**
 * @brief Configures GPIO pins for the mBed shield
 */
void gpio_config(void) {
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);  // Enable the clock for GPIOA (B and C) peripheral
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
}

/**
 * @brief Configure GPIO pins for SCL and SDA, configure and enable I2C 1 clock
 */
void i2c_1_config(void) {
  // Configure SCL as: Alternate function, High Speed, Open Drain, Pull Up
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_8, LL_GPIO_AF_4);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_8, LL_GPIO_OUTPUT_OPENDRAIN);

  // Configure SDA as: Alternate function, High Speed, Open Drain, Pull Up
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_9, LL_GPIO_AF_4);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_OPENDRAIN);

  // Enable I2C 1 Clock
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1); // Enable the clock of I2C1 peripheral
  LL_I2C_Disable(I2C1); // Disable the I2C line before changing its configuration
  LL_I2C_SetMode(I2C1, LL_I2C_MODE_I2C);  // Configure I2C1 peripheral mode to the standard I2C
  LL_I2C_ConfigSpeed(I2C1, 84000000, 100000, LL_I2C_DUTYCYCLE_2); // Set the speed to 100 kHz
  LL_I2C_Enable(I2C1);
}

/**
 * @brief Calculate IPv4 header style checksum for the packet
 * @param packet_type_t Packet for which the checksum has to be calculated
 * @return 16-bit checksum value
 */
uint16_t checksum_calculate(packet_type_t packet) {
    uint16_t words[32] = {0}; // 64 bytes (32 x 16-bit words)

    words[0] = ((packet.dest & 0xffff0000) >> 16);  // 4 destination bytes
    words[1] = (packet.dest & 0xffff);
    words[2] = ((packet.src & 0xffff0000) >> 16);   // 4 source bytes
    words[3] = (packet.src & 0xffff);
    words[4] = packet.length; // 2 length bytes

    for (int i = 0; i < 27; i++) { // 54 payload bytes
      words[5 + i] = packet.payload[i * 2];
      words[5 + i] = ((words[5 + i] << 8) | packet.payload[i * 2 + 1]);
    }

    uint32_t acc = 0; // Use 32-bit accumulator (acc) to sum all 16-bit words
    for (int i = 0; i < 32; i++) { acc += words[i]; }

    while (acc >> 16 != 0x0000) { // Keep folding the carry bits into 16-bit sum until there is no more carry
      acc = (acc & 0x0000ffff) + (acc >> 16);
    }

    uint16_t sum = (uint16_t)(acc); // Cast to 16 bits
    return (~sum); // Return one's complement of the sum
}

/* -------------------------------------------------- */
/* ---------------------- EEPROM -------------------- */

/**
 * @brief Write provided data to the EEPROM in the page write mode
 * @param packet_type_t* Pointer to a packet to be stored in memory
 * @param uint8_tSize of the packet in bytes
 */
void eeprom_write(packet_type_t *packet, uint8_t size) {
  uint8_t byte = 0;	// Currently transmitted byte
	uint8_t page = 0;	// 32 bits per page

  while (byte < size) {
		LL_I2C_GenerateStartCondition(I2C1);	  // I2C Start
  	while (!LL_I2C_IsActiveFlag_SB(I2C1));

		LL_I2C_TransmitData8(I2C1, EEPROMADR);	// Transmit EEPROM address (set R/W to 0, Write)
  	while (!LL_I2C_IsActiveFlag_ADDR(I2C1));
  	LL_I2C_ClearFlag_ADDR(I2C1);

  	LL_I2C_TransmitData8(I2C1, 0x00);				// Transmit Address High Byte (0x00)
  	while (!LL_I2C_IsActiveFlag_TXE(I2C1));

		LL_I2C_TransmitData8(I2C1, (0x00 + page*32));	// Transmit Address Low Byte (increment by 32 bits with each page)
  	while (!LL_I2C_IsActiveFlag_TXE(I2C1));

		uint8_t page_end = 0;

		while(!page_end && (byte < size)) {	  // Transmit batches of 32 bytes until the entire packet is stored in the EEPROM
			uint8_t *next_byte = (uint8_t *)(packet) + byte;
			LL_I2C_TransmitData8(I2C1, *next_byte);  // Store the next byte of the packet
  		while (!LL_I2C_IsActiveFlag_TXE(I2C1));
			byte++;
			if(byte%32 == 0) {page_end = 1;}    // Check if the current page is full
		}

		LL_I2C_GenerateStopCondition(I2C1); 	// I2C Stop
		eeprom_ack_polling();	// Poll for ack before proceeding to the next page
		page++;
	}
}

/**
 * @brief Poll continuously for an acknowledge from EEPROM to check if the device is available for the next operation
 */
void eeprom_ack_polling(void) {
	uint8_t acknowledge_polling_cont = 1; // Continue polling flag
	LL_I2C_ClearFlag_ADDR(I2C1);  // Clear the address ack flag 

	while(acknowledge_polling_cont) {
		LL_I2C_GenerateStartCondition(I2C1);  // I2C Start
  	while (!LL_I2C_IsActiveFlag_SB(I2C1));
		
		LL_I2C_TransmitData8(I2C1, EEPROMADR);	// Transmit EEPROM address (set R/W to 0, Write)
		LL_mDelay(500);

		if(LL_I2C_IsActiveFlag_ADDR(I2C1) == 0) {	// Check if Acknowledge Flag hasn't been set
			acknowledge_polling_cont = 1; // Continue polling
		} else {
			LL_I2C_ClearFlag_ADDR(I2C1);
			acknowledge_polling_cont = 0; // Stop polling
		}
	}
}

/**
 * @brief Read data from the EEPROM
 * @param packet_type_t* Pointer to a packet to be retrived from memory
 * @param uint8_t Size of the packet in bytes
 */
void eeprom_read(packet_type_t* packet, uint8_t size) {
	uint8_t byte = 0;

  LL_I2C_GenerateStartCondition(I2C1);  // I2C Start
  while (!LL_I2C_IsActiveFlag_SB(I2C1));

  LL_I2C_TransmitData8(I2C1, EEPROMADR);  // Transmit EEPROM address (set R/W to 0, Write)
  while (!LL_I2C_IsActiveFlag_ADDR(I2C1));
  LL_I2C_ClearFlag_ADDR(I2C1);

  LL_I2C_TransmitData8(I2C1, 0x00); // Transmit Internal Address High Byte (0x00)
  while (!LL_I2C_IsActiveFlag_TXE(I2C1));

	LL_I2C_TransmitData8(I2C1, (0x00)); // Transmit Internal Address Low Byte (0x00)
  while (!LL_I2C_IsActiveFlag_TXE(I2C1));

	LL_I2C_GenerateStartCondition(I2C1);  // I2C Restart
  while (!LL_I2C_IsActiveFlag_SB(I2C1));

	LL_I2C_TransmitData8(I2C1, (EEPROMADR+1));  // Transmit EEPROM address (set R/W to 1, Read)
  while (!LL_I2C_IsActiveFlag_ADDR(I2C1));
  LL_I2C_ClearFlag_ADDR(I2C1);

  while (byte < size) { // Retrive all the bytes in the packet
    uint8_t *next_byte = (uint8_t *)(packet) + byte;  // Calculate the memory adress of the next byte in the array
    LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK); // ACK Incoming data
    while (!LL_I2C_IsActiveFlag_RXNE(I2C1));
    *next_byte = LL_I2C_ReceiveData8(I2C1); // Store the data received and clears the RXNE flag
    byte++; // Proceed to the next byte
	}

	LL_I2C_GenerateStopCondition(I2C1); // I2C Stop
}

/* -------------------------------------------------- */
/* ----------------------- GUI ---------------------- */

/**
 * @brief Configure lcd pins and SPI 1, clear screen, initialise lcd controller and set font to be used
 */
void lcd_config(void) {
  Configure_LCD_Pins();
  Configure_SPI1();
  Activate_SPI1();
  Clear_Screen();
  Initialise_LCD_Controller();
  set_font((unsigned char *)Arial_9);
}

/**
 * @brief Update the graphical user interface on the LCD based on the state of the program
 */
void gui_update(gui_state_type_t state, packet_type_t packet) {
  switch(state) {
    case GUI_DEST: {
      Clear_Screen();  
      put_string(0, 0, "Destination:");
      char outputString[18]; // Buffer to store text in for LCD
      sprintf(outputString, "0x%x", packet.dest); // Convert data to string
      put_string(0, 15, outputString);
      break;
    }
    case GUI_SRC: {
      Clear_Screen();  
      put_string(0, 0, "Source:");
      char outputString[18]; // Buffer to store text in for LCD
      sprintf(outputString, "0x%x", packet.src); // Convert data to string
      put_string(0, 15, outputString);
      break;
    }
    case GUI_LENGTH: {
      Clear_Screen();  
      put_string(0, 0, "Length:");
      char outputString[18]; // Buffer to store text in for LCD
      sprintf(outputString, "0x%x", packet.length); // Convert data to string
      put_string(0, 15, outputString);
      break;
    }
    case GUI_PAYLOAD: {
      Clear_Screen();
      
      uint8_t d_bin[8]; // Data in binary
      for (int j=0; j<8; j++) { // Converts 8 bits into array for display purposes
        d_bin[j] = (packet.payload[0] & (0x80 >> j)) > 0;
      }      

      put_string(0, 0, "Tilt Reg:");
      char outputString[18]; // Buffer to store text in for LCD
      // Convert data to string
      sprintf(outputString, "%x%x%x%x%x%x%x%x", d_bin[0], d_bin[1], d_bin[2], d_bin[3], d_bin[4], d_bin[5], d_bin[6], d_bin[7]);
      put_string(0, 15, outputString);
      break;
    }
    case GUI_CHECKSUM:{
      Clear_Screen();  
      put_string(0, 0, "Checksum:");
      char outputString[18]; // Buffer to store text in for LCD
      sprintf(outputString, "0x%x", packet.checksum); // Convert data to string
      put_string(0, 15, outputString);
      break;
    }
    case GUI_CHECKSUM_TEST: {
      Clear_Screen();
      if(packet.checksum == packet.recalc_checksum) { // If the checksum is correct
        put_string(0, 0, "Checksum OK:");
        char outputString[18]; // Buffer to store text in for LCD
        sprintf(outputString, "0x%x", packet.checksum); // Convert data to string
        put_string(0, 15, outputString);
      } else {  // If the checksum retvied is different from the recalculated one
        put_string(0, 0, "Checksum err!");
        char outputString[18]; // Buffer to store text in for LCD
        // Display both the retrived (wrong) and recalculated (correct) checksum
        sprintf(outputString, "%x ! %x", packet.checksum, packet.recalc_checksum); // Convert data to string
        put_string(0, 15, outputString);
      }
      break;
    }
    case GUI_SAMPLED: {
      Clear_Screen();
      put_string(0, 0, "Sampled");
      break;
    }
    case GUI_WRITTEN: {
      Clear_Screen();
      put_string(0, 0, "Written");
      break;
    }
    case GUI_RETRIVED: {
      Clear_Screen();
      put_string(0, 0, "Retrived");
      break;
    }
    default: {
      Clear_Screen();
      put_string(0, 0, "Err: state inv");
    }
  }

  LL_mDelay(500000);  // Time for updating the LCD (also joystick debounce timeout)
}

/* -------------------------------------------------- */
/* ------------------ Accelerometer ----------------- */

/**
 * @brief Set the accelerometer to the active mode
 */
void accelerometer_config(void) {
  LL_I2C_GenerateStartCondition(I2C1);  // I2C Start
  while (!LL_I2C_IsActiveFlag_SB(I2C1));

  LL_I2C_TransmitData8(I2C1, ACCELADR); // Transmit accelerometer address (set R/W to 0, Write)
  while (!LL_I2C_IsActiveFlag_ADDR(I2C1));
  LL_I2C_ClearFlag_ADDR(I2C1);

  LL_I2C_TransmitData8(I2C1, 0x07); // Transmit mode register address
  while (!LL_I2C_IsActiveFlag_TXE(I2C1));

  LL_I2C_TransmitData8(I2C1, 0x01); // Set device to active mode (Mode register bit 0 = 1)
  while (!LL_I2C_IsActiveFlag_TXE(I2C1));

  LL_I2C_GenerateStopCondition(I2C1); // I2C Stop
}

/**
 * @brief Read the data stored in the tilt register of the accelerometer
 * @return 8-bit value read from the tilt reg
 */
uint8_t accelerometer_read(void) {
  uint8_t data = 0;

  LL_I2C_GenerateStartCondition(I2C1);  // I2C Start
  while (!LL_I2C_IsActiveFlag_SB(I2C1));

  LL_I2C_TransmitData8(I2C1, ACCELADR); // Transmit accelerometer address (set R/W to 0, Write)
  while (!LL_I2C_IsActiveFlag_ADDR(I2C1));
  LL_I2C_ClearFlag_ADDR(I2C1);

  LL_I2C_TransmitData8(I2C1, 0x03); // Transmit tilt register address
  while (!LL_I2C_IsActiveFlag_TXE(I2C1));

  LL_I2C_GenerateStartCondition(I2C1);  // I2C Restart
  while (!LL_I2C_IsActiveFlag_SB(I2C1));

  LL_I2C_TransmitData8(I2C1, ACCELADR + 1);  // Transmit accelerometer address (set R/W to 1, Read)
  while (!LL_I2C_IsActiveFlag_ADDR(I2C1));
  LL_I2C_ClearFlag_ADDR(I2C1);

  LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);  // NACK Incoming data
  while (!LL_I2C_IsActiveFlag_RXNE(I2C1));
  data = LL_I2C_ReceiveData8(I2C1); // Store the data received and clears the RXNE flag

  LL_I2C_GenerateStopCondition(I2C1); // I2C Stop

  return data;
}

/* -------------------------------------------------- */
/* -------------------- Joystick -------------------- */

/**
 * @brief Configure GPIO pins for the joystick
 */
void joystick_config(void) {
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_INPUT); // set PB5 (centre) as Input
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_5, LL_GPIO_PULL_NO);    // set PB5 as NO pull

  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_INPUT); // set PA4 (up) as Input
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_4, LL_GPIO_PULL_NO);    // set PA4 as NO pull

  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT); // set PB0 (down) as Input
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_0, LL_GPIO_PULL_NO);    // set PB0 as NO pull

  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_1, LL_GPIO_MODE_INPUT); // set PC1 (left) as Input
  LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_1, LL_GPIO_PULL_NO);    // set PC1 as NO pull
  
  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT); // set PC0 (right) as Input
  LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_0, LL_GPIO_PULL_NO);    // set PC0 as NO pull
}

/**
 * @brief Test whether the joystick centre button is pressed
 * @return 1 if the joystick centre is pressed, 0 otherwise
 */
uint8_t joystick_centre(void) {
  return (LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_5));
}

/**
 * @brief Test whether the joystick up button is pressed
 * @return 1 if the joystick up is pressed, 0 otherwise
 */
uint8_t joystick_up(void) {
  return (LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_4));
}

/**
 * @brief Test whether the joystick down button is pressed
 * @return 1 if the joystick down is pressed, 0 otherwise
 */
uint8_t joystick_down(void) {
  return (LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_0));
}

/**
 * @brief Test whether the joystick left button is pressed
 * @return 1 if the joystick left is pressed, 0 otherwise
 */
uint8_t joystick_left(void) {
  return (LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_1));
}

/**
 * @brief Test whether the joystick right button is pressed
 * @return 1 if the joystick right is pressed, 0 otherwise
 */
uint8_t joystick_right(void) {
  return (LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_0));
}
