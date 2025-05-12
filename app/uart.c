/* Original work Copyright 2023 Dual Tachyon
 * https://github.com/DualTachyon
 *
 * Modified work Copyright 2024 kamilsss655
 * https://github.com/kamilsss655
 *
 * Modified work Copyright 2025 dobrishinov
 * https://github.com/dobrishinov
 * Note: I hereby authorize the use of my modifications in this code within the premium firmware,
 * without any limitations on its application, including for closed-source or commercial purposes.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *     Unless required by applicable law or agreed to in writing, software
 *     distributed under the License is distributed on an "AS IS" BASIS,
 *     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *     See the License for the specific language governing permissions and
 *     limitations under the License.
 */

#include <string.h>

#if !defined(ENABLE_OVERLAY)
	#include "ARMCM0.h"
#endif
#ifdef ENABLE_FMRADIO
	#include "app/fm.h"
#endif
#include "app/uart.h"
#include "board.h"
#include "bsp/dp32g030/dma.h"
#include "bsp/dp32g030/gpio.h"
#include "driver/backlight.h"
#include "driver/bk4819.h"
#include "driver/crc.h"
#include "driver/eeprom.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/system.h"
#include "functions.h"
#include "misc.h"
#include "settings.h"
#if defined(ENABLE_OVERLAY)
	#include "sram-overlay.h"
#endif
#include "version.h"
#if defined(ENABLE_MESSENGER) && defined(ENABLE_MESSENGER_UART)
	#include "app/messenger.h"
	#include "external/printf/printf.h"
	#include "ui/ui.h"
	//#include <strings.h>
	//#include <stdlib.h>
	//#include "bsp/dp32g030/syscon.h"
	//#include "driver/adc.h"
#endif


#define DMA_INDEX(x, y) (((x) + (y)) % sizeof(UART_DMA_Buffer))

typedef struct {
	uint16_t ID;
	uint16_t Size;
} Header_t;

typedef struct {
	uint8_t  Padding[2];
	uint16_t ID;
} Footer_t;

typedef struct {
	Header_t Header;
	uint32_t Timestamp;
} CMD_0514_t;

typedef struct {
	Header_t Header;
	struct {
		char     Version[16];
		bool     bHasCustomAesKey;
		bool     bIsInLockScreen;
		uint8_t  Padding[2];
		uint32_t Challenge[4];
	} Data;
} REPLY_0514_t;

typedef struct {
	Header_t Header;
	uint16_t Offset;
	uint8_t  Size;
	uint8_t  Padding;
	uint32_t Timestamp;
} CMD_051B_t;

typedef struct {
	Header_t Header;
	struct {
		uint16_t Offset;
		uint8_t  Size;
		uint8_t  Padding;
		uint8_t  Data[128];
	} Data;
} REPLY_051B_t;

typedef struct {
	Header_t Header;
	uint16_t Offset;
	uint8_t  Size;
	bool     bAllowPassword;
	uint32_t Timestamp;
	uint8_t  Data[0];
} CMD_051D_t;

typedef struct {
	Header_t Header;
	struct {
		uint16_t Offset;
	} Data;
} REPLY_051D_t;

typedef struct {
	Header_t Header;
	struct {
		uint16_t RSSI;
		uint8_t  ExNoiseIndicator;
		uint8_t  GlitchIndicator;
	} Data;
} REPLY_0527_t;

typedef struct {
	Header_t Header;
	struct {
		uint16_t Voltage;
		uint16_t Current;
	} Data;
} REPLY_0529_t;

typedef struct {
	Header_t Header;
	struct {
		bool bIsLocked;
		uint8_t Padding[3];
	} Data;
} REPLY_052D_t;

typedef struct {
	Header_t Header;
	uint32_t Timestamp;
} CMD_052F_t;

static const uint8_t Obfuscation[16] =
{
	0x16, 0x6C, 0x14, 0xE6, 0x2E, 0x91, 0x0D, 0x40, 0x21, 0x35, 0xD5, 0x40, 0x13, 0x03, 0xE9, 0x80
};

static union
{
	uint8_t Buffer[256];
	struct
	{
		Header_t Header;
		uint8_t Data[252];
	};
} UART_Command;

static uint32_t Timestamp;
static uint16_t gUART_WriteIndex;
#if defined(ENABLE_MESSENGER) && defined(ENABLE_MESSENGER_UART)
static uint16_t gUART_SMSWriteIndex;
#endif
static bool     bIsEncrypted = true;

static void SendReply(void *pReply, uint16_t Size)
{
	Header_t Header;
	Footer_t Footer;

	if (bIsEncrypted)
	{
		uint8_t     *pBytes = (uint8_t *)pReply;
		unsigned int i;
		for (i = 0; i < Size; i++)
			pBytes[i] ^= Obfuscation[i % 16];
	}

	Header.ID = 0xCDAB;
	Header.Size = Size;
	UART_Send(&Header, sizeof(Header));
	UART_Send(pReply, Size);

	if (bIsEncrypted)
	{
		Footer.Padding[0] = Obfuscation[(Size + 0) % 16] ^ 0xFF;
		Footer.Padding[1] = Obfuscation[(Size + 1) % 16] ^ 0xFF;
	}
	else
	{
		Footer.Padding[0] = 0xFF;
		Footer.Padding[1] = 0xFF;
	}
	Footer.ID = 0xBADC;

	UART_Send(&Footer, sizeof(Footer));
}

static void SendVersion(void)
{
	REPLY_0514_t Reply;

	Reply.Header.ID = 0x0515;
	Reply.Header.Size = sizeof(Reply.Data);
	strcpy(Reply.Data.Version, Version);
	Reply.Data.bHasCustomAesKey = bHasCustomAesKey;
	Reply.Data.bIsInLockScreen = bIsInLockScreen;
	Reply.Data.Challenge[0] = gChallenge[0];
	Reply.Data.Challenge[1] = gChallenge[1];
	Reply.Data.Challenge[2] = gChallenge[2];
	Reply.Data.Challenge[3] = gChallenge[3];

	SendReply(&Reply, sizeof(Reply));
}

static void CMD_0514(const uint8_t *pBuffer)
{
	const CMD_0514_t *pCmd = (const CMD_0514_t *)pBuffer;

	Timestamp = pCmd->Timestamp;

	#ifdef ENABLE_FMRADIO
		gFmRadioCountdown_500ms = fm_radio_countdown_500ms;
	#endif

	gSerialConfigCountDown_500ms = 12; // 6 sec

	// turn the LCD backlight off
	BACKLIGHT_TurnOff();

	SendVersion();
}

static void CMD_051B(const uint8_t *pBuffer)
{
	const CMD_051B_t *pCmd = (const CMD_051B_t *)pBuffer;
	REPLY_051B_t      Reply;
	bool              bLocked = false;

	if (pCmd->Timestamp != Timestamp)
		return;

	gSerialConfigCountDown_500ms = 12; // 6 sec

	#ifdef ENABLE_FMRADIO
		gFmRadioCountdown_500ms = fm_radio_countdown_500ms;
	#endif

	memset(&Reply, 0, sizeof(Reply));
	Reply.Header.ID   = 0x051C;
	Reply.Header.Size = pCmd->Size + 4;
	Reply.Data.Offset = pCmd->Offset;
	Reply.Data.Size   = pCmd->Size;

	if (bHasCustomAesKey)
		bLocked = gIsLocked;

	if (!bLocked)
		EEPROM_ReadBuffer(pCmd->Offset, Reply.Data.Data, pCmd->Size);

	SendReply(&Reply, pCmd->Size + 8);
}

static void CMD_051D(const uint8_t *pBuffer)
{
	const CMD_051D_t *pCmd = (const CMD_051D_t *)pBuffer;
	REPLY_051D_t Reply;
	bool bReloadEeprom;
	bool bIsLocked;

	if (pCmd->Timestamp != Timestamp)
		return;

	gSerialConfigCountDown_500ms = 12; // 6 sec

	bReloadEeprom = false;

	#ifdef ENABLE_FMRADIO
		gFmRadioCountdown_500ms = fm_radio_countdown_500ms;
	#endif

	Reply.Header.ID   = 0x051E;
	Reply.Header.Size = sizeof(Reply.Data);
	Reply.Data.Offset = pCmd->Offset;

	bIsLocked = bHasCustomAesKey ? gIsLocked : bHasCustomAesKey;

	if (!bIsLocked)
	{
		unsigned int i;
		for (i = 0; i < (pCmd->Size / 8); i++)
		{
			const uint16_t Offset = pCmd->Offset + (i * 8U);

			if (Offset >= 0x0F30 && Offset < 0x0F40)
				if (!gIsLocked)
					bReloadEeprom = true;

			if ((Offset < 0x0E98 || Offset >= 0x0EA0) || !bIsInLockScreen || pCmd->bAllowPassword)
				EEPROM_WriteBuffer(Offset, &pCmd->Data[i * 8U], true);
		}

		if (bReloadEeprom)
			BOARD_EEPROM_Init();
	}

	SendReply(&Reply, sizeof(Reply));
}

static void CMD_0527(void)
{
	REPLY_0527_t Reply;

	Reply.Header.ID             = 0x0528;
	Reply.Header.Size           = sizeof(Reply.Data);
	Reply.Data.RSSI             = BK4819_ReadRegister(BK4819_REG_67) & 0x01FF;
	Reply.Data.ExNoiseIndicator = BK4819_ReadRegister(BK4819_REG_65) & 0x007F;
	Reply.Data.GlitchIndicator  = BK4819_ReadRegister(BK4819_REG_63);

	SendReply(&Reply, sizeof(Reply));
}

static void CMD_0529(void)
{
	REPLY_0529_t Reply;

	Reply.Header.ID   = 0x52A;
	Reply.Header.Size = sizeof(Reply.Data);

	// Original doesn't actually send current!
	BOARD_ADC_GetBatteryInfo(&Reply.Data.Voltage);

	SendReply(&Reply, sizeof(Reply));
}

static void CMD_052F(const uint8_t *pBuffer)
{
	const CMD_052F_t *pCmd = (const CMD_052F_t *)pBuffer;

	gEeprom.DUAL_WATCH                               = DUAL_WATCH_OFF;
	gEeprom.CROSS_BAND_RX_TX                         = CROSS_BAND_OFF;
	gEeprom.RX_VFO                                   = 0;
	gEeprom.DTMF_SIDE_TONE                           = false;
	gEeprom.VfoInfo[0].FrequencyReverse              = false;
	gEeprom.VfoInfo[0].pRX                           = &gEeprom.VfoInfo[0].freq_config_RX;
	gEeprom.VfoInfo[0].pTX                           = &gEeprom.VfoInfo[0].freq_config_TX;
	gEeprom.VfoInfo[0].TX_OFFSET_FREQUENCY_DIRECTION = TX_OFFSET_FREQUENCY_DIRECTION_OFF;
	gEeprom.VfoInfo[0].DTMF_PTT_ID_TX_MODE           = PTT_ID_OFF;
#ifdef ENABLE_DTMF_CALLING
	gEeprom.VfoInfo[0].DTMF_DECODING_ENABLE          = false;
#endif

	#ifdef ENABLE_NOAA
		gIsNoaaMode = false;
	#endif

	if (gCurrentFunction == FUNCTION_POWER_SAVE)
		FUNCTION_Select(FUNCTION_FOREGROUND);

	gSerialConfigCountDown_500ms = 12; // 6 sec

	Timestamp = pCmd->Timestamp;

	// turn the LCD backlight off
	BACKLIGHT_TurnOff();

	SendVersion();
}

#if defined(ENABLE_MESSENGER) && defined(ENABLE_MESSENGER_UART)
// UART_PrintBufferSlice is a helper function to print: DMA Buffer Content
void UART_PrintBufferSlice(const char* label, const char* buffer, size_t startIndex, size_t length) {
	UART_printf("%s[", label);
	for (size_t i = 0; i < length; ++i) {
		char c = buffer[DMA_INDEX(startIndex, i)];
		if (c >= 32 && c <= 126) {
			UART_printf("%c", c); // printable ASCII
		} else {
			UART_printf(".");     // unprintable shown as dot
		}
	}
	UART_printf("], BufferIndex: [%d/256] \r\n", startIndex);
}

bool commandStartsWith(const char *message, const char *command, int length) {
    for (int i = 0; i < length; i++) {
        char msgChar = message[i];
        char cmdChar = command[i];

        // Convert both characters to lowercase for case-insensitive comparison
        if (msgChar >= 'A' && msgChar <= 'Z') {
            msgChar += 32; // Convert to lowercase
        }
        if (cmdChar >= 'A' && cmdChar <= 'Z') {
            cmdChar += 32; // Convert to lowercase
        }

        if (msgChar != cmdChar) {
            return false;
        }
    }
    return true;
}

void UART_IsSMSAvailable(void)
{
    uint16_t DmaLength = DMA_CH0->ST & 0xFFFU;

    while (1)
    {
        if (gUART_SMSWriteIndex == DmaLength)
            break;

        // Check if the message starts with "@" (command) or "SMS:" (text message)
		bool isCommand = (UART_DMA_Buffer[gUART_SMSWriteIndex] == '@');

        if (isCommand || strncmp(((char*)UART_DMA_Buffer) + gUART_SMSWriteIndex, "SMS:", 4) == 0)
        {
            //UART_PrintBufferSlice("[UART Message]", (char*)UART_DMA_Buffer, gUART_SMSWriteIndex, PAYLOAD_LENGTH + 4);

            char txMessage[PAYLOAD_LENGTH + 1]; // +1 for null-terminator
            memset(txMessage, 0, sizeof(txMessage));

            // Extract the message content
            size_t copyLength = ExtractMessageContent(txMessage, gUART_SMSWriteIndex, isCommand);

            // Process the extracted message
            if (isCommand)
            {
                if (ProcessCommand(txMessage, copyLength))
                {
                    // Command was processed, clear the buffer and update the write index
                    ClearBufferAndUpdateIndex();
                    continue;
                }
                else
                {
                    // If the message starts with "@" but is not a valid command, ignore it
                    UART_printf("INV COM: %s\r\n", txMessage);
                    ClearBufferAndUpdateIndex();
                    continue;
                }
            }

            // If not a command, send the message
            if (copyLength > 0)
            {
                SendMessage(txMessage);
            }

            // Clear the buffer and update the write index
            ClearBufferAndUpdateIndex();
        }

        gUART_SMSWriteIndex = DmaLength;
        break;
    }
}

/**
 * Extracts the message content from the UART DMA buffer.
 */
size_t ExtractMessageContent(char *txMessage, uint16_t startIndex, bool isCommand)
{
    size_t copyLength = 0;

	// Adjust the starting index based on whether it's a command or a text message
    uint16_t adjustedIndex = isCommand ? startIndex : startIndex + 4;

    for (size_t i = 0; i < PAYLOAD_LENGTH; i++)
    {
        char c = UART_DMA_Buffer[DMA_INDEX(adjustedIndex, i)];

        // Stop at end of message markers or non-printable ASCII characters
        if (c == '\0' || c == '\r' || c == '\n' || c < 32 || c > 126)
            break;

        txMessage[copyLength++] = c;
    }

    // Ensure null-termination
    txMessage[copyLength] = '\0';

    return copyLength;
}

/**
 * Processes a command from the extracted message.
 * Returns true if the message was a command, false otherwise.
 */
bool ProcessCommand(const char *txMessage, size_t copyLength)
{
    if (commandStartsWith(txMessage, "@RP ON", 6))
    {
        repeaterMode = true;
        msgAutoRetryEnabled = false;
        UART_printf("RP:ON\r\n");
        return true;
    }
    else if (commandStartsWith(txMessage, "@RP OFF", 7))
    {
        repeaterMode = false;
        UART_printf("RP:OFF\r\n");
        return true;
    }
    else if (commandStartsWith(txMessage, "@AR ON", 6))
    {
        msgAutoRetryEnabled = true;
        repeaterMode = false;
        UART_printf("AR:ON\r\n");
        return true;
    }
    else if (commandStartsWith(txMessage, "@AR OFF", 7))
    {
        msgAutoRetryEnabled = false;
        UART_printf("AR:OFF\r\n");
        return true;
    }
    else if (commandStartsWith(txMessage, "@F ", 3))
    {
        HandleFrequencyCommand(txMessage, copyLength);
        return true;
    }
	else if (commandStartsWith(txMessage, "@ID", 3))
    {	
		// Print the unique ID
		char uniqueId[7]; // Allocate a buffer for the unique ID
		BOARD_GetDeviceUniqueId(uniqueId);
		UART_printf("ID:%s\r\n", uniqueId);
		//UART_printf("Device Unique ID: %s\r\n", uniqueId);

        return true;
    }
#ifdef ENABLE_TEMP_SENSOR
	else if (commandStartsWith(txMessage, "@T", 6))
    {
		// Convert to whole degrees and print the rounded temperature
		UART_printf("%d°C\r\n", BOARD_GetDeviceTemperature());

        return true;
    }
#endif

    return false;
}

/**
 * Handles the "@F" frequency change command.
 */
void HandleFrequencyCommand(const char *txMessage, size_t copyLength)
{
    uint32_t Frequency = 0;
    bool isGigaF = gTxVfo->pRX->Frequency >= 100000000;

    // Ensure the input length is valid for frequency
    if (copyLength < (size_t)(6 + isGigaF))
    {
        UART_printf("ERR:FREQ LEN\r\n");
        return;
    }

    // Parse the frequency from the command
    const char *ptr = txMessage + 3;
    while (*ptr >= '0' && *ptr <= '9')
    {
        Frequency = Frequency * 10 + (*ptr - '0');
        ptr++;
    }

    Frequency *= 100; // Multiply the result by 100

    // Validate and set the frequency
    if (Frequency >= RX_freq_min() && RX_freq_check(Frequency) == 0)
    {
        SETTINGS_SetVfoFrequency(Frequency);
        gRequestSaveChannel = 1;
        UART_printf("FREQ SET %u MHz\r\n", Frequency);
        gRequestDisplayScreen = DISPLAY_MAIN;
        gUpdateDisplay = true;
    }
    else
    {
        UART_printf("ERR:FREQ INV %u MHz\r\n", Frequency);
    }
}

/**
 * Sends a message if it is not a command.
 */
void SendMessage(const char *txMessage)
{
    isMsgReceived = 0;
    msgStatus = READY; // Prevent glitches with rx_finished
	msgRetryCount = 0;
    MSG_Send(txMessage);
    gUpdateDisplay = true;
}

/**
 * Clears the UART DMA buffer and updates the write index.
 */
void ClearBufferAndUpdateIndex()
{
    for (size_t i = 0; i < PAYLOAD_LENGTH + 4; i++)
    {
        UART_DMA_Buffer[DMA_INDEX(gUART_SMSWriteIndex, i)] = 0;
    }

    gUART_SMSWriteIndex = DMA_INDEX(gUART_SMSWriteIndex, PAYLOAD_LENGTH + 4);
}

// void UART_IsSMSAvailable(void)
// {
// 	uint16_t DmaLength = DMA_CH0->ST & 0xFFFU;

// 	while (1)
// 	{
// 		if (gUART_SMSWriteIndex == DmaLength)
// 			break;

// 		// Check for "SMS:"
//         if (strncmp(((char*)UART_DMA_Buffer) + gUART_SMSWriteIndex, "SMS:", 4) == 0)
//         {
// 			UART_PrintBufferSlice("[UART Message]", (char*)UART_DMA_Buffer, gUART_SMSWriteIndex, PAYLOAD_LENGTH + 4);

// 			char txMessage[PAYLOAD_LENGTH + 1]; // +1 for null-terminator
// 			memset(txMessage, 0, sizeof(txMessage));

// 			// Extract message after "SMS:" prefix
// 			size_t copyLength = 0;
// 			for (size_t i = 0; i < PAYLOAD_LENGTH; i++) {
// 				char c = UART_DMA_Buffer[DMA_INDEX(gUART_SMSWriteIndex + 4, i)];

// 			// Stop at end of message markers or non-printable ASCII characters
// 			if (c == '\0' || c == '\r' || c == '\n' || c < 32 || c > 126) {
// 				break;
// 			}

// 				txMessage[copyLength++] = c;
// 			}

// 			// Ensure null-termination (redundant due to memset, but safe)
// 			txMessage[copyLength] = '\0';

// 			// Process commands
//             bool isCommand = false;
//             if (commandStartsWith(txMessage, "@RP ON", 6)) {
// 				repeaterMode = true;
// 				msgAutoRetryEnabled = false;
// 				UART_printf("Repeater Mode Enabled\r\n");
// 				isCommand = true;
// 			} else if (commandStartsWith(txMessage, "@RP OFF", 7)) {
// 				repeaterMode = false;
// 				UART_printf("Repeater Mode Disabled\r\n");
// 				isCommand = true;
// 			} else if (commandStartsWith(txMessage, "@AR ON", 6)) {
// 				msgAutoRetryEnabled = true;
// 				repeaterMode = false;
// 				UART_printf("Auto Retry Enabled\r\n");
// 				isCommand = true;
// 			} else if (commandStartsWith(txMessage, "@AR OFF", 7)) {
// 				msgAutoRetryEnabled = false;
// 				UART_printf("Auto Retry Disabled\r\n");
// 				isCommand = true;
// 			} else if (commandStartsWith(txMessage, "@F ", 3)) {
//                 // Handle frequency change command
// 				uint32_t Frequency;
// 				bool isGigaF = gTxVfo->pRX->Frequency >= 100000000;

// 				// Ensure the input length is valid for frequency
// 				if (copyLength < (size_t)(6 + isGigaF)) {
// 					UART_printf("Invalid Frequency Input Length\r\n");
// 					return;
// 				}

// 				// Parse the frequency from the command
// 				//Frequency = strtoul(txMessage + 3, NULL, 10) * 100;

// 				Frequency = 0;

// 				// Assuming `txMessage + 3` points to the number part in the string
// 				char *ptr = txMessage + 3;
// 				while (*ptr >= '0' && *ptr <= '9')  // Check if the character is a digit
// 				{
// 					Frequency = Frequency * 10 + (*ptr - '0');  // Convert char to digit and accumulate
// 					ptr++;
// 				}

// 				Frequency *= 100;  // Multiply the result by 100

// 				// Validate and set the frequency
// 				if (Frequency >= RX_freq_min() && RX_freq_check(Frequency) == 0) {
// 					SETTINGS_SetVfoFrequency(Frequency); // Update the frequency
// 					gRequestSaveChannel = 1;            // Mark the channel for saving
// 					UART_printf("Frequency Set to %u MHz\r\n", Frequency);
// 					gRequestDisplayScreen = DISPLAY_MAIN;
// 					gUpdateDisplay = true;
// 				} else {
// 					UART_printf("Invalid Frequency: %u MHz\r\n", Frequency);
// 				}

// 				isCommand = true;
//             }

// 			// Only send if there's actual content and it's not a recognized command
//             if (copyLength > 0 && !isCommand) {
// 				isMsgReceived = 0;
// 				msgStatus = READY; //Adding this to prevent the glitch with rx_finished that sometime is not triggered 
// 				MSG_Send(txMessage);
// 				/*
// 				* In order to print message to the Serial when we also type message from keyboard 
// 				* I moved the UART_printf("SMS>%s\r\n", txMessage); to MSG_SendPacket method in app/messenger.c
// 				*/
// 				//UART_printf("SMS>%s\r\n", txMessage);
// 				gUpdateDisplay = true;
// 			}

// 			// Debug log before clearing
// 			//UART_printf("Debug: Clearing Buffer from WriteIndex: %d, Length: %d\r\n", gUART_SMSWriteIndex, PAYLOAD_LENGTH + 4);

// 			// Clear full region even if copyLength was short (to avoid leftover data)
// 			for (size_t i = 0; i < PAYLOAD_LENGTH + 4; i++) {
// 				UART_DMA_Buffer[DMA_INDEX(gUART_SMSWriteIndex, i)] = 0;
// 			}
			
// 			// Debug log before Before Update WriteIndex
// 			//UART_printf("Debug: Before Update WriteIndex: %d\r\n", gUART_SMSWriteIndex);

// 			// Update write index
// 			gUART_SMSWriteIndex = DMA_INDEX(gUART_SMSWriteIndex, PAYLOAD_LENGTH + 4); // Always skip 4 + max payload

// 			// Debug log after Before Update WriteIndex
// 			//UART_printf("Debug: After Update WriteIndex: %d\r\n", gUART_SMSWriteIndex);
// 		}

// 		gUART_SMSWriteIndex = DmaLength;

// 		break; 
// 	}
// }
#endif

bool UART_IsCommandAvailable(void)
{
	uint16_t Index;
	uint16_t TailIndex;
	uint16_t Size;
	uint16_t CRC;
	uint16_t CommandLength;
	uint16_t DmaLength = DMA_CH0->ST & 0xFFFU;

	while (1)
	{
		if (gUART_WriteIndex == DmaLength)
			return false;
		
		while (gUART_WriteIndex != DmaLength && UART_DMA_Buffer[gUART_WriteIndex] != 0xABU)
			gUART_WriteIndex = DMA_INDEX(gUART_WriteIndex, 1);

		if (gUART_WriteIndex == DmaLength)
			return false;

		if (gUART_WriteIndex < DmaLength)
			CommandLength = DmaLength - gUART_WriteIndex;
		else
			CommandLength = (DmaLength + sizeof(UART_DMA_Buffer)) - gUART_WriteIndex;

		if (CommandLength < 8)
			return 0;

		if (UART_DMA_Buffer[DMA_INDEX(gUART_WriteIndex, 1)] == 0xCD)
			break;

		gUART_WriteIndex = DMA_INDEX(gUART_WriteIndex, 1);
	}

	Index = DMA_INDEX(gUART_WriteIndex, 2);
	Size  = (UART_DMA_Buffer[DMA_INDEX(Index, 1)] << 8) | UART_DMA_Buffer[Index];

	if ((Size + 8u) > sizeof(UART_DMA_Buffer))
	{
		gUART_WriteIndex = DmaLength;
		return false;
	}

	if (CommandLength < (Size + 8))
		return false;

	Index     = DMA_INDEX(Index, 2);
	TailIndex = DMA_INDEX(Index, Size + 2);

	if (UART_DMA_Buffer[TailIndex] != 0xDC || UART_DMA_Buffer[DMA_INDEX(TailIndex, 1)] != 0xBA)
	{
		gUART_WriteIndex = DmaLength;
		return false;
	}

	if (TailIndex < Index)
	{
		const uint16_t ChunkSize = sizeof(UART_DMA_Buffer) - Index;
		memmove(UART_Command.Buffer, UART_DMA_Buffer + Index, ChunkSize);
		memmove(UART_Command.Buffer + ChunkSize, UART_DMA_Buffer, TailIndex);
	}
	else
		memmove(UART_Command.Buffer, UART_DMA_Buffer + Index, TailIndex - Index);

	TailIndex = DMA_INDEX(TailIndex, 2);
	if (TailIndex < gUART_WriteIndex)
	{
		memset(UART_DMA_Buffer + gUART_WriteIndex, 0, sizeof(UART_DMA_Buffer) - gUART_WriteIndex);
		memset(UART_DMA_Buffer, 0, TailIndex);
	}
	else
		memset(UART_DMA_Buffer + gUART_WriteIndex, 0, TailIndex - gUART_WriteIndex);

	gUART_WriteIndex = TailIndex;

	if (UART_Command.Header.ID == 0x0514)
		bIsEncrypted = false;

	if (UART_Command.Header.ID == 0x6902)
		bIsEncrypted = true;

	if (bIsEncrypted)
	{
		unsigned int i;
		for (i = 0; i < (Size + 2u); i++)
			UART_Command.Buffer[i] ^= Obfuscation[i % 16];
	}

	CRC = UART_Command.Buffer[Size] | (UART_Command.Buffer[Size + 1] << 8);

	return (CRC_Calculate(UART_Command.Buffer, Size) != CRC) ? false : true;
}

void UART_HandleCommand(void)
{
	switch (UART_Command.Header.ID)
	{
		case 0x0514:
			CMD_0514(UART_Command.Buffer);
			break;

		case 0x051B:
			CMD_051B(UART_Command.Buffer);
			break;

		case 0x051D:
			CMD_051D(UART_Command.Buffer);
			break;

		case 0x051F:	// Not implementing non-authentic command
			break;

		case 0x0521:	// Not implementing non-authentic command
			break;

		case 0x0527:
			CMD_0527();
			break;

		case 0x0529:
			CMD_0529();
			break;

		case 0x052F:
			CMD_052F(UART_Command.Buffer);
			break;

		case 0x05DD:
			#if defined(ENABLE_OVERLAY)
				overlay_FLASH_RebootToBootloader();
			#else
				NVIC_SystemReset();
			#endif
			break;
	}
}
