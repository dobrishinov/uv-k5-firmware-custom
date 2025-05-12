/* Original work Copyright 2023 joaquimorg
 * https://github.com/joaquimorg
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

#ifdef ENABLE_MESSENGER

#include <string.h>
#include "driver/keyboard.h"
#include "driver/st7565.h"
#include "driver/bk4819.h"
#include "external/printf/printf.h"
#include "misc.h"
#include "settings.h"
#include "radio.h"
#include "app.h"
#include "audio.h"
#include "functions.h"
#include "frequencies.h"
#include "driver/system.h"
#include "app/messenger.h"
#include "ui/ui.h"
#ifdef ENABLE_ENCRYPTION
	#include "helper/crypto.h"
#endif
#ifdef ENABLE_MESSENGER_UART
    #include "driver/uart.h"
#endif
#ifdef ENABLE_MESSENGER_CRC
	#include "driver/crc.h"
#endif
#include "board.h"

const uint8_t MSG_BUTTON_STATE_HELD = 1 << 1;

const uint8_t MSG_BUTTON_EVENT_SHORT =  0;
const uint8_t MSG_BUTTON_EVENT_LONG =  MSG_BUTTON_STATE_HELD;

const uint8_t MAX_MSG_LENGTH = PAYLOAD_LENGTH_LIMITED;

uint16_t TONE2_FREQ;

#define NEXT_CHAR_DELAY 100 // 10ms tick

char T9TableLow[9][5] = { {',', '.', '?', '!', '/'}, {'a', 'b', 'c', '\0', '\0'}, {'d', 'e', 'f', '\0', '\0'}, {'g', 'h', 'i', '\0', '\0'}, {'j', 'k', 'l', '\0', '\0'}, {'m', 'n', 'o', '\0', '\0'}, {'p', 'q', 'r', 's', '\0'}, {'t', 'u', 'v', '\0', '\0'}, {'w', 'x', 'y', 'z', '\0'} };
char T9TableUp[9][5] = { {'+', '-', ':', '#', '='}, {'A', 'B', 'C', '\0', '\0'}, {'D', 'E', 'F', '\0', '\0'}, {'G', 'H', 'I', '\0', '\0'}, {'J', 'K', 'L', '\0', '\0'}, {'M', 'N', 'O', '\0', '\0'}, {'P', 'Q', 'R', 'S', '\0'}, {'T', 'U', 'V', '\0', '\0'}, {'W', 'X', 'Y', 'Z', '\0'} };
unsigned char numberOfLettersAssignedToKey[9] = { 5, 3, 3, 3, 3, 3, 4, 3, 4 };

char T9TableNum[9][4] = { {'1', '\0', '\0', '\0'}, {'2', '\0', '\0', '\0'}, {'3', '\0', '\0', '\0'}, {'4', '\0', '\0', '\0'}, {'5', '\0', '\0', '\0'}, {'6', '\0', '\0', '\0'}, {'7', '\0', '\0', '\0'}, {'8', '\0', '\0', '\0'}, {'9', '\0', '\0', '\0'} };
unsigned char numberOfNumsAssignedToKey[9] = { 1, 1, 1, 1, 1, 1, 1, 1, 1 };

char cMessage[PAYLOAD_LENGTH_LIMITED + 1];
char lastcMessage[PAYLOAD_LENGTH_LIMITED + 1];
char rxMessage[40][PAYLOAD_LENGTH_LIMITED + 3];
unsigned char cIndex = 0;
unsigned char prevKey = 0, prevLetter = 0;
KeyboardType keyboardType = UPPERCASE;
KEY_Code_t keyboardKey = KEY_INVALID;

MsgStatus msgStatus = READY;

union DataPacket dataPacket;

uint16_t gErrorsDuringMSG;

uint8_t hasNewMessage = 0;

uint8_t keyTickCounter = 0;

uint8_t isMsgReceived = 0;

char copiedMessage[PAYLOAD_LENGTH_LIMITED + 1] = {0}; // Buffer for copied text

uint8_t optionsButtonsTextState = 0;

uint8_t currentPage = 0;

uint8_t msgRetryCount = 1;
bool msgWaitingForAck = false;
uint32_t msgLastSendTimestamp = 0;
char msgRetryBuffer[PAYLOAD_LENGTH + 1] = {0};
bool msgAutoRetryEnabled = false;
uint8_t msgAutoRetryPopup;

bool repeaterMode = false;
uint32_t lastPttPressTimestamp = 0;

RepeaterState repeaterState = REPEATER_IDLE;
uint32_t repeaterDelayStart = 0;

char lastDirectSenderId[8] = {0}; // 7 chars + null

uint32_t lastBeaconTick = 0;
uint8_t beaconState = 0; // 0 = joined, 1 = alive, 2 = RP started, 3 = RP online

// Persistent buffer for repeater retransmission
char repeater_originalSenderId[7] = {0};
uint8_t repeater_originalPayload[PAYLOAD_LENGTH] = {0};
uint8_t repeater_originalHeader = 0;

// -----------------------------------------------------

void MSG_FSKSendData() {

	// turn off CTCSS/CDCSS during FFSK
	const uint16_t css_val = BK4819_ReadRegister(BK4819_REG_51);
	BK4819_WriteRegister(BK4819_REG_51, 0);

	// set the FM deviation level
	const uint16_t dev_val = BK4819_ReadRegister(BK4819_REG_40);

	{
		uint16_t deviation;
		switch (gEeprom.VfoInfo[gEeprom.TX_VFO].CHANNEL_BANDWIDTH)
		{
			case BK4819_FILTER_BW_WIDE:            deviation =  1300; break; // 20k // measurements by kamilsss655
			case BK4819_FILTER_BW_NARROW:          deviation =  1200; break; // 10k
			// case BK4819_FILTER_BW_NARROWAVIATION:  deviation =  850; break;  // 5k
			// case BK4819_FILTER_BW_NARROWER:        deviation =  850; break;  // 5k
			// case BK4819_FILTER_BW_NARROWEST:	      deviation =  850; break;  // 5k
			default:                               deviation =  850;  break;  // 5k
		}

		//BK4819_WriteRegister(0x40, (3u << 12) | (deviation & 0xfff));
		BK4819_WriteRegister(BK4819_REG_40, (dev_val & 0xf000) | (deviation & 0xfff));
	}

	// REG_2B   0
	//
	// <15> 1 Enable CTCSS/CDCSS DC cancellation after FM Demodulation   1 = enable 0 = disable
	// <14> 1 Enable AF DC cancellation after FM Demodulation            1 = enable 0 = disable
	// <10> 0 AF RX HPF 300Hz filter     0 = enable 1 = disable
	// <9>  0 AF RX LPF 3kHz filter      0 = enable 1 = disable
	// <8>  0 AF RX de-emphasis filter   0 = enable 1 = disable
	// <2>  0 AF TX HPF 300Hz filter     0 = enable 1 = disable
	// <1>  0 AF TX LPF filter           0 = enable 1 = disable
	// <0>  0 AF TX pre-emphasis filter  0 = enable 1 = disable
	//
	// disable the 300Hz HPF and FM pre-emphasis filter
	//
	const uint16_t filt_val = BK4819_ReadRegister(BK4819_REG_2B);
	BK4819_WriteRegister(BK4819_REG_2B, (1u << 2) | (1u << 0));
	
	MSG_ConfigureFSK(false);



	SYSTEM_DelayMs(100);

	{	// load the entire packet data into the TX FIFO buffer
		for (size_t i = 0, j = 0; i < sizeof(dataPacket.serializedArray); i += 2, j++) {
        	BK4819_WriteRegister(BK4819_REG_5F, (dataPacket.serializedArray[i + 1] << 8) | dataPacket.serializedArray[i]);
    	}
	}

	// enable FSK TX
	BK4819_FskEnableTx();

	{
		// allow up to 310ms for the TX to complete
		// if it takes any longer then somethings gone wrong, we shut the TX down
		unsigned int timeout = 1000 / 5;

		while (timeout-- > 0)
		{
			SYSTEM_DelayMs(5);
			if (BK4819_ReadRegister(BK4819_REG_0C) & (1u << 0))
			{	// we have interrupt flags
				BK4819_WriteRegister(BK4819_REG_02, 0);
				if (BK4819_ReadRegister(BK4819_REG_02) & BK4819_REG_02_FSK_TX_FINISHED)
					timeout = 0;       // TX is complete
			}
		}
	}
	//BK4819_WriteRegister(BK4819_REG_02, 0);

	SYSTEM_DelayMs(100);

	// disable TX
	MSG_ConfigureFSK(true);

	// restore FM deviation level
	BK4819_WriteRegister(BK4819_REG_40, dev_val);

	// restore TX/RX filtering
	BK4819_WriteRegister(BK4819_REG_2B, filt_val);

	// restore the CTCSS/CDCSS setting
	BK4819_WriteRegister(BK4819_REG_51, css_val);

}

void MSG_EnableRX(const bool enable) {

	if (enable) {
		MSG_ConfigureFSK(true);

		if(gEeprom.MESSENGER_CONFIG.data.receive)
			BK4819_FskEnableRx();
	} else {
		BK4819_WriteRegister(BK4819_REG_70, 0);
		BK4819_WriteRegister(BK4819_REG_58, 0);
	}
}


// -----------------------------------------------------

void navigatePages(bool up) {
    if (up) {
        if (currentPage > 0) {
            currentPage--;
        }
    } else {
        if (currentPage < 7) { // Maximum 8 pages (0 to 7)
            currentPage++;
        }
    }
    gUpdateDisplay = true; // Trigger display update
}

void moveUP(char (*rxMessages)[PAYLOAD_LENGTH_LIMITED + 3]) {
    // Shift all messages up within the array
    for (int i = 0; i < 39; i++) {
        strcpy(rxMessages[i], rxMessages[i + 1]);
    }

    // Clear the last message slot
    memset(rxMessages[39], 0, sizeof(rxMessages[39]));
}

void MSG_SendPacket() {

	if ( msgStatus != READY ) return;

	RADIO_PrepareTX();

	if(RADIO_GetVfoState() != VFO_STATE_NORMAL){
		gRequestDisplayScreen = DISPLAY_MAIN;
		return;
	} 

	if ( strlen((char *)dataPacket.data.payload) > 0) {

		msgStatus = SENDING;

		RADIO_SetVfoState(VFO_STATE_NORMAL);
		BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, false);
		BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, true);

		// display sent message (before encryption)
		// Only move up and add a new line if this is the first send (not a retry)
        if (msgRetryCount == 1 && dataPacket.data.header != ACK_PACKET) {
            moveUP(rxMessage);
			//sprintf(rxMessage[3], "> %s", dataPacket.data.payload);
			
			snprintf(rxMessage[39], PAYLOAD_LENGTH_LIMITED + 3, dataPacket.data.recipientId[0] != '\0' ? "> %s:%s" : "> %s%s", dataPacket.data.recipientId, dataPacket.data.payload);
		} else if (dataPacket.data.header != ACK_PACKET) {
		    // On retry, update the last line to indicate retry only in AUTO REPLY mode
			if (msgAutoRetryEnabled && !repeaterMode) {
				snprintf(rxMessage[39], PAYLOAD_LENGTH_LIMITED + 3, "> (%d)%s:%s", msgRetryCount, dataPacket.data.recipientId, dataPacket.data.payload);
			} else {
				// Default behavior for other modes
				snprintf(rxMessage[39], PAYLOAD_LENGTH_LIMITED + 3, "> %s", dataPacket.data.payload);
			}
        }
		
			#ifdef ENABLE_MESSENGER_UART
			if (dataPacket.data.header != ACK_PACKET) {
				char uart_buf[PAYLOAD_LENGTH + 1];
				memcpy(uart_buf, dataPacket.data.payload, PAYLOAD_LENGTH);
				uart_buf[PAYLOAD_LENGTH] = '\0'; // Ensure null-termination
				//UART_printf("SMS>%s\r\n", uart_buf);
				if (dataPacket.data.recipientId[0] != '\0') {
					UART_printf(">TX #D#%s#%s\r\n", dataPacket.data.recipientId, uart_buf);
				} else {
					UART_printf(">TX #A#%s\r\n", uart_buf);
				}
			}
			#endif

			// Only update lastcMessage for non-ACK packets
			if (dataPacket.data.header != ACK_PACKET) {
				memset(lastcMessage, 0, sizeof(lastcMessage));
				memcpy(lastcMessage, dataPacket.data.payload, PAYLOAD_LENGTH_LIMITED);
				cIndex = 0;
				prevKey = 0;
				prevLetter = 0;
				memset(cMessage, 0, sizeof(cMessage));
			}

		#ifdef ENABLE_ENCRYPTION
			if(dataPacket.data.header == ENCRYPTED_MESSAGE_PACKET){

				CRYPTO_Random(dataPacket.data.nonce, NONCE_LENGTH);

				CRYPTO_Crypt(
					dataPacket.data.payload,
					PAYLOAD_LENGTH_LIMITED,
					dataPacket.data.payload,
					&dataPacket.data.nonce,
					gEncryptionKey,
					256
				);
			}
		#endif

		BK4819_DisableDTMF();

		// mute the mic during TX
		gMuteMic = true;

		SYSTEM_DelayMs(50);

		MSG_FSKSendData();

		SYSTEM_DelayMs(50);

		APP_EndTransmission(false);
		// this must be run after end of TX, otherwise radio will still TX transmit without even RED LED on
		FUNCTION_Select(FUNCTION_FOREGROUND);

		RADIO_SetVfoState(VFO_STATE_NORMAL);

		// disable mic mute after TX
		gMuteMic = false;

		BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, false);

		MSG_EnableRX(true);

		// clear packet buffer
		MSG_ClearPacketBuffer();

		msgStatus = READY;

	} else {
		AUDIO_PlayBeep(BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL);
		msgStatus = READY; // TODO: BUGFIX
	}
}

void MSG_StorePacket(const uint16_t interrupt_bits) {

	//const uint16_t rx_sync_flags   = BK4819_ReadRegister(BK4819_REG_0B);

	const bool rx_sync             = (interrupt_bits & BK4819_REG_02_FSK_RX_SYNC) ? true : false;
	const bool rx_fifo_almost_full = (interrupt_bits & BK4819_REG_02_FSK_FIFO_ALMOST_FULL) ? true : false;
	const bool rx_finished         = (interrupt_bits & BK4819_REG_02_FSK_RX_FINISHED) ? true : false;

	//UART_printf("\nMSG : S%i, F%i, E%i | %i", rx_sync, rx_fifo_almost_full, rx_finished, interrupt_bits);

	if (rx_sync) {
		#ifdef ENABLE_MESSENGER_FSK_MUTE
			// prevent listening to fsk data and squelch (kamilsss655)
			// CTCSS codes seem to false trigger the rx_sync
			if(gCurrentCodeType == CODE_TYPE_OFF)
				AUDIO_AudioPathOff();
		#endif
		gFSKWriteIndex = 0;
		MSG_ClearPacketBuffer();
		msgStatus = RECEIVING;
	}

	if (rx_fifo_almost_full && msgStatus == RECEIVING) {

		const uint16_t count = BK4819_ReadRegister(BK4819_REG_5E) & (7u << 0);  // almost full threshold
		for (uint16_t i = 0; i < count; i++) {
			const uint16_t word = BK4819_ReadRegister(BK4819_REG_5F);
			if (gFSKWriteIndex < sizeof(dataPacket.serializedArray))
				dataPacket.serializedArray[gFSKWriteIndex++] = (word >> 0) & 0xff;
			if (gFSKWriteIndex < sizeof(dataPacket.serializedArray))
				dataPacket.serializedArray[gFSKWriteIndex++] = (word >> 8) & 0xff;
		}

		SYSTEM_DelayMs(10);

	}

	if (rx_finished) {
		// turn off green LED
		BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, 0);
		BK4819_FskClearFifo();
		BK4819_FskEnableRx();
		msgStatus = READY;

		if (gFSKWriteIndex > 2) {
			MSG_HandleReceive();
		}
		gFSKWriteIndex = 0;
	}
}

void MSG_SendSystemBeacon(const char *prefix) {
    char msg[48];
    int8_t temp = BOARD_GetDeviceTemperature();
    int8_t batt = BATTERY_VoltsToPercent(gBatteryVoltageAverage);
    snprintf(msg, sizeof(msg), "%s T:%dC B:%d%%", prefix, temp, batt);
    MSG_Send(msg);
}

void MSG_BeaconTask(void) {
	if (!gEeprom.MESSENGER_CONFIG.data.beacon)
		return;

    uint32_t now = gGlobalSysTickCounter;

    if (beaconState == 0) {
        // On startup
        MSG_SendSystemBeacon(repeaterMode ? "RP Started" : "Joined");
        lastBeaconTick = now;
        beaconState = repeaterMode ? 2 : 1;
        return;
    }

    if (repeaterMode) {
        // Repeater mode: send "RP Online" every 60 minutes (1 min = 600 * 10ms)
        if (now - lastBeaconTick > 360000) { // 60 min = 60 * 600 * 10ms
            MSG_SendSystemBeacon("RP Online");
            lastBeaconTick = now;
        }
    } else {
        // Normal mode: send "Alive" every 30 minutes (1 min = 600 * 10ms)
        if (now - lastBeaconTick > 180000) { // 30 min = 30 * 600 * 10ms
            MSG_SendSystemBeacon("Alive");
            lastBeaconTick = now;
        }
    }
}

void MSG_Init() {
	memset(rxMessage, 0, sizeof(rxMessage));
	memset(cMessage, 0, sizeof(cMessage));
	memset(lastcMessage, 0, sizeof(lastcMessage));
	hasNewMessage = 0;
	msgStatus = READY;
	prevKey = 0;
    prevLetter = 0;
	cIndex = 0;
	currentPage = 7; // Default to the last page
	#ifdef ENABLE_ENCRYPTION
		gRecalculateEncKey = true;
	#endif

	// --- Send "Online" join message with temp and battery ---
    // char joinMsg[40];
    // int temp = BOARD_GetDeviceTemperature();
    // int batt = BATTERY_VoltsToPercent(gBatteryVoltageAverage);
    // snprintf(joinMsg, sizeof(joinMsg), "Online T:%dC B:%d%%", temp, batt);
    // MSG_Send(joinMsg);
}

void MSG_SendAck() {
	// Send ACK packet to the sender
	char ackTo[8];
	memcpy(ackTo, dataPacket.data.senderId, 7);
	ackTo[7] = 0;
	
	MSG_ClearPacketBuffer();
	
	dataPacket.data.header = ACK_PACKET;
	BOARD_GetDeviceUniqueId(dataPacket.data.senderId);
	memcpy(dataPacket.data.recipientId, ackTo, 8);

	// sending only empty header seems to not work, so set few bytes of payload to increase reliability (kamilsss655)
	memset(dataPacket.data.payload, 255, 5);

	#ifdef ENABLE_MESSENGER_CRC
    // Calculate CRC for the ACK payload
    uint16_t crc = CRC_Calculate(dataPacket.data.payload, PAYLOAD_LENGTH);
    dataPacket.data.crc[0] = (crc >> 8) & 0xFF;
    dataPacket.data.crc[1] = crc & 0xFF;
	#endif
	
	MSG_SendPacket();
}

void MSG_HandleReceive(){
	char myId[7];
    BOARD_GetDeviceUniqueId(myId);

	// Save the original payload for possible retransmission
    uint8_t originalPayload[PAYLOAD_LENGTH];
    memcpy(originalPayload, dataPacket.data.payload, PAYLOAD_LENGTH);

	// Only accept if recipientId is empty or matches myId
    bool isForMe = (dataPacket.data.recipientId[0] == '\0') ||
                   (strncmp(dataPacket.data.recipientId, myId, 6) == 0);

    if (!isForMe) {
        msgStatus = READY;
        return;
	}

	#ifdef ENABLE_MESSENGER_CRC
	// --- Simulate corruption for testing ---
    //dataPacket.data.payload[0] ^= 0xFF; // Flip all bits of the first byte (simulate error)
    // --- End of simulation ---

	// --- CRC: Check CRC before delivering ---
    uint16_t received_crc = (dataPacket.data.crc[0] << 8) | dataPacket.data.crc[1];
    uint16_t calc_crc = CRC_Calculate(dataPacket.data.payload, PAYLOAD_LENGTH);

    if (received_crc != calc_crc) {
        // CRC mismatch, do not deliver or ACK
        moveUP(rxMessage);
        snprintf(rxMessage[39], PAYLOAD_LENGTH_LIMITED + 3, "! ERROR: CRC FAILED!");
		#ifdef ENABLE_MESSENGER_UART
			UART_printf("ERR:CRC\r\n");
		#endif
        gUpdateDisplay = true;
		msgStatus = READY; // TODO: BUGFIX
        return;
    }
    // --- CRC OK, continue as before ---
	#endif

	// Ignore ACK_PACKET in Repeater mode to prevent loops
    if (repeaterMode && dataPacket.data.header == ACK_PACKET) {
        msgStatus = READY; // Ignore ACK and reset status
        return;
    }

	if (dataPacket.data.header == ACK_PACKET) {
		// Only process ACK if it is for me
		if (strncmp(dataPacket.data.recipientId, myId, 6) != 0) {
            msgStatus = READY;
            return;
        }
		//("ACK_PACKET #%02X#%s#%s#%s\r\n", dataPacket.data.header, dataPacket.data.senderId, dataPacket.data.recipientId, dataPacket.data.payload);
	#ifdef ENABLE_MESSENGER_DELIVERY_NOTIFICATION
		#ifdef ENABLE_MESSENGER_UART
			UART_printf("<ACK #D#%s#\r\n", dataPacket.data.senderId);
		#endif
		rxMessage[39][0] = '+';
		isMsgReceived = 1;
		gUpdateStatus = true;
		gUpdateDisplay = true;
	#endif
		msgWaitingForAck = false;
		msgRetryCount = 1;
		msgStatus = READY; // TODO: BUGFIX
	} else {
		// Check for retransmission loop using the repeater flag
		if (repeaterMode && (dataPacket.data.header & 0x80)) {
			// Message already retransmitted by a repeater, ignore it
			msgStatus = READY;
			return;
		}

		moveUP(rxMessage);
		// Mask out the repeater flag before comparing
		if ((dataPacket.data.header & 0x7F) >= INVALID_PACKET) {
			//dataPacket.data.payload[PAYLOAD_LENGTH_LIMITED] = '\0';
			snprintf(rxMessage[39], PAYLOAD_LENGTH_LIMITED + 3, "! ERROR: INVALID PACKET!");
			#ifdef ENABLE_MESSENGER_UART
				UART_printf("ERR:INV PACKET\r\n");
			#endif
			msgStatus = READY; // TODO: BUGFIX
			return; //TODO: Check again, Not sure do we need return here?
		}
		else
		{
			#ifdef ENABLE_ENCRYPTION
				if(dataPacket.data.header == ENCRYPTED_MESSAGE_PACKET)
				{
					CRYPTO_Crypt(dataPacket.data.payload,
						PAYLOAD_LENGTH_LIMITED,
						dataPacket.data.payload,
						&dataPacket.data.nonce,
						gEncryptionKey,
						256);
				}
			#endif
			
			// Check if the message starts with "RP:" and skip the prefix if present
			if (dataPacket.data.header & 0x80) {
				//snprintf(rxMessage[39], PAYLOAD_LENGTH_LIMITED + 3, "R %s", dataPacket.data.payload + 3);
				snprintf(rxMessage[39], PAYLOAD_LENGTH_LIMITED + 3, "R %s", dataPacket.data.payload);
			} else {
				//snprintf(rxMessage[39], PAYLOAD_LENGTH_LIMITED + 3, "< %s", dataPacket.data.payload);
				snprintf(rxMessage[39], 
						 PAYLOAD_LENGTH_LIMITED + 3, 
						 (strncmp(dataPacket.data.recipientId, myId, 6) == 0) ? "D %s:%s" : "< %s:%s", dataPacket.data.senderId, dataPacket.data.payload);
				
				if ((strncmp(dataPacket.data.recipientId, myId, 6) == 0)) {
					// Save senderId for reply
					strncpy(lastDirectSenderId, dataPacket.data.senderId, 7);
					lastDirectSenderId[7] = '\0';
				}
			}

			#ifdef ENABLE_MESSENGER_UART
				char uart_buf[PAYLOAD_LENGTH + 1];
				memcpy(uart_buf, dataPacket.data.payload, PAYLOAD_LENGTH);
				uart_buf[PAYLOAD_LENGTH] = '\0';
				if (dataPacket.data.header & 0x80) {
					// Message was retransmitted by a repeater
					UART_printf("<RX #R#%s\r\n", uart_buf);
				} else {
					//UART_printf("<RX #%s#%s\r\n", dataPacket.data.senderId, uart_buf);
					UART_printf((strncmp(dataPacket.data.recipientId, myId, 6) == 0) ? "<RX #D#%s#%s\r\n" : "<RX #A#%s#%s\r\n", dataPacket.data.senderId, uart_buf);
				}
			#endif

			isMsgReceived = 0;
			hasNewMessage = 1;
		}

		// Automatically scroll to the last page
        currentPage = 7; // Page index starts from 0, so page 8 is index 7

		if ( gScreenToDisplay != DISPLAY_MSG ) {
			hasNewMessage = 1;
			gUpdateStatus = true;
			gUpdateDisplay = true;
	#ifdef ENABLE_MESSENGER_NOTIFICATION
			gPlayMSGRing = true;
	#endif
		}
		else {
			gUpdateDisplay = true;
		}
	}

	// Retransmit the message in repeater mode
    if (repeaterMode && !(dataPacket.data.header & 0x80)) {
        // Save the original sender and payload for delayed retransmission
        memcpy(repeater_originalSenderId, dataPacket.data.senderId, 7);
        memcpy(repeater_originalPayload, originalPayload, PAYLOAD_LENGTH);
        repeater_originalHeader = dataPacket.data.header;

        // Set the repeater state to waiting
        repeaterState = REPEATER_WAITING;
        repeaterDelayStart = gGlobalSysTickCounter; // Record the start time
    }

	// Transmit a message to the sender that we have received the message
	if (dataPacket.data.header == MESSAGE_PACKET ||
		dataPacket.data.header == ENCRYPTED_MESSAGE_PACKET)
	{
		// wait so the correspondent radio can properly receive it
		SYSTEM_DelayMs(700);

		if(!repeaterMode && gEeprom.MESSENGER_CONFIG.data.ack && dataPacket.data.recipientId[0] != '\0')
			MSG_SendAck();
	}
}

void MSG_HandleRepeaterState(void) {

	//TODO: EXCTRACT RANDOM DELAY TO FUNCTION
    static uint32_t randomDelay = 0;
    static uint32_t lfsr = 0xACE1u; // Seed for pseudo-random

    if (repeaterState == REPEATER_WAITING) {
        if (randomDelay == 0) {
            // Simple LFSR pseudo-random generator
            lfsr ^= lfsr << 13;
            lfsr ^= lfsr >> 9;
            lfsr ^= lfsr << 7;
            // Use lower 8 bits for random delay.
			// Modulo (% 12) ensures 12 possible values (0–11). 
			// Multiplying by 150 gives increments of 0 ms, 150 ms, 300 ms, … up to 1650 ms.
            randomDelay = ((lfsr ^ gGlobalSysTickCounter) % 12) * 150;
			UART_printf("RPWait#%dms\n", randomDelay);
        }
        if ((gGlobalSysTickCounter - repeaterDelayStart) >= (150 + randomDelay)) {
            repeaterState = REPEATER_TRANSMIT;
            randomDelay = 0;
        }
    }

	// if (repeaterState == REPEATER_WAITING) {
    //     // Check if the delay has elapsed
    //     if ((gGlobalSysTickCounter - repeaterDelayStart) >= 500) { // 5 second delay
    //         repeaterState = REPEATER_TRANSMIT;
    //     }
    // }

    if (repeaterState == REPEATER_TRANSMIT) {
		// Prepare retransmission using the saved original message
        MSG_ClearPacketBuffer();
        dataPacket.data.header = repeater_originalHeader | 0x80; // set repeater flag
        BOARD_GetDeviceUniqueId(dataPacket.data.senderId);
        memset(dataPacket.data.recipientId, 0, 7); // broadcast
        snprintf((char*)dataPacket.data.payload, PAYLOAD_LENGTH, "%s#%s:%s",
                 dataPacket.data.senderId, repeater_originalSenderId, repeater_originalPayload);

        #ifdef ENABLE_MESSENGER_CRC
        uint16_t crc = CRC_Calculate(dataPacket.data.payload, PAYLOAD_LENGTH);
        dataPacket.data.crc[0] = (crc >> 8) & 0xFF;
        dataPacket.data.crc[1] = crc & 0xFF;
        #endif

        // Transmit the message
        MSG_SendPacket();
        msgStatus = READY; // Reset the message status
        repeaterState = REPEATER_IDLE; // Reset the state
    }
}

// ---------------------------------------------------------------------------------

void insertCharInMessage(uint8_t key) {
	if ( key == KEY_0 ) {
		if ( keyboardType == NUMERIC ) {
			cMessage[cIndex] = '0';
		} else if ( keyboardType == UPPERCASE) {
			cMessage[cIndex] = ' ';
		} else {
			cMessage[cIndex] = ' ';
		}
		if ( cIndex < MAX_MSG_LENGTH ) {
			cIndex++;
		}
	} else if (prevKey == key)
	{
		cIndex = (cIndex > 0) ? cIndex - 1 : 0;
		if ( keyboardType == NUMERIC ) {
			cMessage[cIndex] = T9TableNum[key - 1][(++prevLetter) % numberOfNumsAssignedToKey[key - 1]];
		} else if ( keyboardType == LOWERCASE ) {
			cMessage[cIndex] = T9TableLow[key - 1][(++prevLetter) % numberOfLettersAssignedToKey[key - 1]];
		} else {
			cMessage[cIndex] = T9TableUp[key - 1][(++prevLetter) % numberOfLettersAssignedToKey[key - 1]];
		}
		if ( cIndex < MAX_MSG_LENGTH ) {
			cIndex++;
		}
	}
	else
	{
		prevLetter = 0;
		if ( cIndex > MAX_MSG_LENGTH ) {
			cIndex = (cIndex > 0) ? cIndex - 1 : 0;
		}
		if ( keyboardType == NUMERIC ) {
			cMessage[cIndex] = T9TableNum[key - 1][prevLetter];
		} else if ( keyboardType == LOWERCASE ) {
			cMessage[cIndex] = T9TableLow[key - 1][prevLetter];
		} else {
			cMessage[cIndex] = T9TableUp[key - 1][prevLetter];
		}
		if ( cIndex < MAX_MSG_LENGTH ) {
			cIndex++;
		}

	}
	cMessage[cIndex] = '\0';
	if ( keyboardType == NUMERIC ) {
		prevKey = 0;
		prevLetter = 0;
	} else {
		prevKey = key;
	}
}

void processBackspace() {
	cIndex = (cIndex > 0) ? cIndex - 1 : 0;
	cMessage[cIndex] = '\0';
	prevKey = 0;
    prevLetter = 0;
}

void  MSG_ProcessKeys(KEY_Code_t Key, bool bKeyPressed, bool bKeyHeld) {
	keyboardKey = Key;
	
	uint8_t state = bKeyPressed + 2 * bKeyHeld;

	if (state == MSG_BUTTON_EVENT_SHORT) {

		switch (Key)
		{
			case KEY_0:
			case KEY_1:
			case KEY_2:
			case KEY_3:
			case KEY_4:
			case KEY_5:
			case KEY_6:
			case KEY_7:
			case KEY_8:
			case KEY_9:
				if ( keyTickCounter > NEXT_CHAR_DELAY) {
					prevKey = 0;
    				prevLetter = 0;
				}
				insertCharInMessage(Key);
				keyTickCounter = 0;

				// // Check for commands
                // if (strcmp(cMessage, "/RP ON") == 0) {
                //     repeaterMode = true;
                //     msgAutoRetryPopup = 1; // Show "Repeater ON" on UI
                //     memset(cMessage, 0, sizeof(cMessage)); // Clear the command
                // } else if (strcmp(cMessage, "/RP OFF") == 0) {
                //     repeaterMode = false;
                //     msgAutoRetryPopup = 2; // Show "Repeater OFF" on UI
                //     memset(cMessage, 0, sizeof(cMessage)); // Clear the command
                // } else if (strcmp(cMessage, "/AR ON") == 0) {
                //     msgAutoRetryEnabled = true;
                //     msgAutoRetryPopup = 1; // Show "Auto Retry ON" on UI
                //     memset(cMessage, 0, sizeof(cMessage)); // Clear the command
                // } else if (strcmp(cMessage, "/AR OFF") == 0) {
                //     msgAutoRetryEnabled = false;
                //     msgAutoRetryPopup = 2; // Show "Auto Retry OFF" on UI
                //     memset(cMessage, 0, sizeof(cMessage)); // Clear the command
                // }

				break;
			case KEY_STAR:
				keyboardType = (KeyboardType)((keyboardType + 1) % END_TYPE_KBRD);
				break;
			case KEY_F:
				processBackspace();
				break;
			case KEY_UP:
				memset(cMessage, 0, sizeof(cMessage));
				memcpy(cMessage, lastcMessage, PAYLOAD_LENGTH_LIMITED);
				cIndex = strlen(cMessage);
				optionsButtonsTextState = 4;
				break;
			/*case KEY_DOWN:
				break;*/
			case KEY_MENU:
				// Fixing issue when we click Menu without text.
				msgStatus = READY; //Adding this to prevent the glitch with rx_finished that sometime is not triggered 
				if (strlen(cMessage) > 0) {
					msgRetryCount = 0;
					// Send message
					MSG_Send(cMessage);
				}
				break;
			case KEY_EXIT:
				gRequestDisplayScreen = DISPLAY_MAIN;
				break;

			case KEY_DOWN:
				if (strlen(copiedMessage) > 0) {
					// Paste the copied text into the input box
					memset(cMessage, 0, sizeof(cMessage));
					strncpy(cMessage, copiedMessage, PAYLOAD_LENGTH_LIMITED);
					cMessage[PAYLOAD_LENGTH_LIMITED] = '\0'; // Ensure null termination
					cIndex = strlen(cMessage);
					optionsButtonsTextState = 1;
					AUDIO_PlayBeep(BEEP_1KHZ_60MS_OPTIONAL); // Optional feedback
				}
                break;
			
			case KEY_SIDE1: // Scroll UP
                navigatePages(true);
                break;
            case KEY_SIDE2: // Scroll DOWN
                navigatePages(false);
                break;

			case KEY_PTT:
                // if (gGlobalSysTickCounter - lastPttPressTimestamp < 50) {
                //     // Rotate through the four options
                //     static uint8_t optionIndex = 0;
                //     optionIndex = (optionIndex + 1) % 4;
                //     switch (optionIndex) {
                //         case 0:
                //             msgAutoRetryPopup = 1; // Show "Repeater ON" on UI
                //             repeaterMode = true;
                //             break;
                //         case 1:
                //             msgAutoRetryPopup = 2; // Show "Repeater OFF" on UI
                //             repeaterMode = false;
                //             break;
                //         case 2:
                //             msgAutoRetryPopup = 3; // Show "Auto Retry ON" on UI
                //             msgAutoRetryEnabled = true;
                //             break;
                //         case 3:
                //             msgAutoRetryPopup = 4; // Show "Auto Retry OFF" on UI
                //             msgAutoRetryEnabled = false;
                //             break;
                //     }
                // }
                // lastPttPressTimestamp = gGlobalSysTickCounter;
                // break;

				if (gGlobalSysTickCounter - lastPttPressTimestamp < 50) { // 50ms for double press
					// Rotate between NORMAL, AUTO REPLY, and REPEATER modes
					static uint8_t modeIndex = 0; // 0: NORMAL, 1: AUTO REPLY, 2: REPEATER
					modeIndex = (modeIndex + 1) % 3;
			
					switch (modeIndex) {
						case 0: // NORMAL mode
							optionsButtonsTextState = 0;
							msgAutoRetryPopup = 3; // Show "MODE: NORMAL" on UI
							repeaterMode = false;
							msgAutoRetryEnabled = false;
							beaconState = 0; // Reset beacon state
							break;
						case 1: // AUTO REPLY mode
							optionsButtonsTextState = 0;
							msgAutoRetryPopup = 1; // Show "MODE: AUTO REPLY" on UI
							repeaterMode = false;
							msgAutoRetryEnabled = true;
							break;
						case 2: // REPEATER mode
							optionsButtonsTextState = 0;
							msgAutoRetryPopup = 2; // Show "MODE: REPEATER" on UI
							repeaterMode = true;
							msgAutoRetryEnabled = false;
							beaconState = 0; // Reset beacon state
							break;
					}
				}
				lastPttPressTimestamp = gGlobalSysTickCounter;
				break;

                //msgAutoRetryEnabled = !msgAutoRetryEnabled;
                //msgAutoRetryPopup = msgAutoRetryEnabled ? 1 : 2;
                //break;

			default:
				AUDIO_PlayBeep(BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL);
				break;
		}

	} else if (state == MSG_BUTTON_EVENT_LONG) {

		switch (Key)
		{
			case KEY_F:
				MSG_Init();
				break;

			case KEY_DOWN:
                // Copy the current input text to the copiedMessage buffer
				if (strlen(cMessage) > 0) {
					strncpy(copiedMessage, cMessage, PAYLOAD_LENGTH_LIMITED);
					copiedMessage[PAYLOAD_LENGTH_LIMITED] = '\0'; // Ensure null termination
					optionsButtonsTextState = 2;
					AUDIO_PlayBeep(BEEP_1KHZ_60MS_OPTIONAL); // Optional feedback
				}
				break;

			case KEY_UP:
                // Prepopulate with #senderId# if available
                if (lastDirectSenderId[0] != '\0') {
                    snprintf(cMessage, PAYLOAD_LENGTH_LIMITED, "#%s#", lastDirectSenderId);
                    cIndex = strlen(cMessage);
                    optionsButtonsTextState = 3;
                    AUDIO_PlayBeep(BEEP_1KHZ_60MS_OPTIONAL);
                }
                break;
			default:
				AUDIO_PlayBeep(BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL);
				break;
		}
	}

}

void MSG_ClearPacketBuffer()
{
	memset(dataPacket.serializedArray, 0, sizeof(dataPacket.serializedArray));
}

// static void parse_recipient_and_payload(const char *msg, char *recipient, char *payload) {
//     memset(recipient, 0, 7);
//     if (msg[0] == '#' && strlen(msg) > 2) {
//         const char *second = strchr(msg + 1, '#');
//         size_t id_len = second ? (size_t)(second - (msg + 1)) : 0;
//         if (second && id_len > 0 && id_len <= 6 && *(second + 1) != '\0') {
//             memcpy(recipient, msg + 1, id_len);
//             recipient[id_len] = 0;
//             strncpy(payload, second + 1, PAYLOAD_LENGTH);
//             return;
//         }
//     }
//     // Fallback: treat as broadcast
//     memset(recipient, 0, 7);
//     strncpy(payload, msg, PAYLOAD_LENGTH);
// }

void MSG_Send(const char *cMessage){
	MSG_ClearPacketBuffer();

	// Set sender ID
    BOARD_GetDeviceUniqueId(dataPacket.data.senderId);

	// parse_recipient_and_payload(cMessage, dataPacket.data.recipientId, (char*)dataPacket.data.payload);

    // #ifdef ENABLE_ENCRYPTION
    //     dataPacket.data.header = gEeprom.MESSENGER_CONFIG.data.encrypt ? ENCRYPTED_MESSAGE_PACKET : MESSAGE_PACKET;
    // #else
    //     dataPacket.data.header = MESSAGE_PACKET;
    // #endif

    // Parse recipient ID if message starts with #ID#
    if (cMessage[0] == '#' && strlen(cMessage) > 2) {
        const char *second_hash = strchr(cMessage + 1, '#');
        if (second_hash && (second_hash - (cMessage + 1)) > 0 && (second_hash - (cMessage + 1)) <= 6) {
            size_t id_len = second_hash - (cMessage + 1);
            if (id_len > 6) id_len = 6;
			// Check if there is a payload after the second #
            if (*(second_hash + 1) == '\0') {
                // No payload, do not send
                AUDIO_PlayBeep(BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL); // Optional feedback
                return;
            }
            strncpy(dataPacket.data.recipientId, cMessage + 1, id_len);
            dataPacket.data.recipientId[id_len] = '\0';
            // Copy payload after the second #
            strncpy((char *)dataPacket.data.payload, second_hash + 1, PAYLOAD_LENGTH);
        } else {
            // Invalid format, treat as broadcast
            memset(dataPacket.data.recipientId, 0, 7);
            strncpy((char *)dataPacket.data.payload, cMessage, PAYLOAD_LENGTH);
        }
    } else {
        memset(dataPacket.data.recipientId, 0, 7);
        strncpy((char *)dataPacket.data.payload, cMessage, PAYLOAD_LENGTH);
    }


	#ifdef ENABLE_ENCRYPTION
		if(gEeprom.MESSENGER_CONFIG.data.encrypt)
		{
			dataPacket.data.header=ENCRYPTED_MESSAGE_PACKET;
		}
		else
		{
			dataPacket.data.header=MESSAGE_PACKET;
		}
	#else
		dataPacket.data.header=MESSAGE_PACKET;
	#endif
	//memcpy(dataPacket.data.payload, cMessage, sizeof(dataPacket.data.payload));
	//memcpy(dataPacket.data.payload, cMessage, PAYLOAD_LENGTH);
	#ifdef ENABLE_MESSENGER_CRC
	// --- CRC: Calculate and append CRC to the end of payload ---
    uint16_t crc = CRC_Calculate(dataPacket.data.payload, PAYLOAD_LENGTH);
    dataPacket.data.crc[0] = (crc >> 8) & 0xFF;
    dataPacket.data.crc[1] = crc & 0xFF;
    // ----------------------------------------------------------
	#endif

	strncpy(msgRetryBuffer, cMessage, PAYLOAD_LENGTH);
    msgRetryBuffer[PAYLOAD_LENGTH] = '\0';

	if (msgRetryCount == 0) msgRetryCount = 1;// Only reset on first send

    msgWaitingForAck = true;
    msgLastSendTimestamp = 0; // will be set on first send

    MSG_SendPacket();
    msgLastSendTimestamp = gGlobalSysTickCounter; // millisecond tick function
}

void MSG_HandleRetryTask(void) {
	if (!msgAutoRetryEnabled)
        return;
    if (!msgWaitingForAck)
        return;

    // 5 seconds = 5000 ms
    uint32_t now = gGlobalSysTickCounter;
    if (now - msgLastSendTimestamp < 500){
        return;
	}

	if (msgRetryCount < 3) {
		msgRetryCount++;
		MSG_Send(msgRetryBuffer); // This will handle parsing, encryption, CRC, etc.
		msgLastSendTimestamp = now;
	} else {
        // Give up after 3 tries
        msgWaitingForAck = false;
		msgStatus = READY; // TODO: BUGFIX
        //UART_printf("Message delivery failed after 3 retries.\r\n");
    }
}

void MSG_ConfigureFSK(bool rx)
{
	// REG_70
	//
	// <15>   0 Enable TONE1
	//        1 = Enable
	//        0 = Disable
	//
	// <14:8> 0 TONE1 tuning gain
	//        0 ~ 127
	//
	// <7>    0 Enable TONE2
	//        1 = Enable
	//        0 = Disable
	//
	// <6:0>  0 TONE2/FSK tuning gain
	//        0 ~ 127
	//
	BK4819_WriteRegister(BK4819_REG_70,
		( 0u << 15) |    // 0
		( 0u <<  8) |    // 0
		( 1u <<  7) |    // 1
		(96u <<  0));    // 96

	// Tone2 = FSK baudrate                       // kamilsss655 2024
	switch(gEeprom.MESSENGER_CONFIG.data.modulation)
	{
		case MOD_AFSK_1200:
			TONE2_FREQ = 12389u;
			break;
		case MOD_FSK_700:
			TONE2_FREQ = 7227u;
			break;
		case MOD_FSK_450:
			TONE2_FREQ = 4646u;
			break;
	}

	BK4819_WriteRegister(BK4819_REG_72, TONE2_FREQ);
	
	switch(gEeprom.MESSENGER_CONFIG.data.modulation)
	{
		case MOD_FSK_700:
		case MOD_FSK_450:
			BK4819_WriteRegister(BK4819_REG_58,
				(0u << 13) |		// 1 FSK TX mode selection
									//   0 = FSK 1.2K and FSK 2.4K TX .. no tones, direct FM
									//   1 = FFSK 1200 / 1800 TX
									//   2 = ???
									//   3 = FFSK 1200 / 2400 TX
									//   4 = ???
									//   5 = NOAA SAME TX
									//   6 = ???
									//   7 = ???
									//
				(0u << 10) |		// 0 FSK RX mode selection
									//   0 = FSK 1.2K, FSK 2.4K RX and NOAA SAME RX .. no tones, direct FM
									//   1 = ???
									//   2 = ???
									//   3 = ???
									//   4 = FFSK 1200 / 2400 RX
									//   5 = ???
									//   6 = ???
									//   7 = FFSK 1200 / 1800 RX
									//
				(3u << 8) |			// 0 FSK RX gain
									//   0 ~ 3
									//
				(0u << 6) |			// 0 ???
									//   0 ~ 3
									//
				(0u << 4) |			// 0 FSK preamble type selection
									//   0 = 0xAA or 0x55 due to the MSB of FSK sync byte 0
									//   1 = ???
									//   2 = 0x55
									//   3 = 0xAA
									//
				(0u << 1) |			// 1 FSK RX bandwidth setting
									//   0 = FSK 1.2K .. no tones, direct FM
									//   1 = FFSK 1200 / 1800
									//   2 = NOAA SAME RX
									//   3 = ???
									//   4 = FSK 2.4K and FFSK 1200 / 2400
									//   5 = ???
									//   6 = ???
									//   7 = ???
									//
				(1u << 0));			// 1 FSK enable
									//   0 = disable
									//   1 = enable
		break;
		case MOD_AFSK_1200:
			BK4819_WriteRegister(BK4819_REG_58,
				(1u << 13) |		// 1 FSK TX mode selection
									//   0 = FSK 1.2K and FSK 2.4K TX .. no tones, direct FM
									//   1 = FFSK 1200 / 1800 TX
									//   2 = ???
									//   3 = FFSK 1200 / 2400 TX
									//   4 = ???
									//   5 = NOAA SAME TX
									//   6 = ???
									//   7 = ???
									//
				(7u << 10) |		// 0 FSK RX mode selection
									//   0 = FSK 1.2K, FSK 2.4K RX and NOAA SAME RX .. no tones, direct FM
									//   1 = ???
									//   2 = ???
									//   3 = ???
									//   4 = FFSK 1200 / 2400 RX
									//   5 = ???
									//   6 = ???
									//   7 = FFSK 1200 / 1800 RX
									//
				(3u << 8) |			// 0 FSK RX gain
									//   0 ~ 3
									//
				(0u << 6) |			// 0 ???
									//   0 ~ 3
									//
				(0u << 4) |			// 0 FSK preamble type selection
									//   0 = 0xAA or 0x55 due to the MSB of FSK sync byte 0
									//   1 = ???
									//   2 = 0x55
									//   3 = 0xAA
									//
				(1u << 1) |			// 1 FSK RX bandwidth setting
									//   0 = FSK 1.2K .. no tones, direct FM
									//   1 = FFSK 1200 / 1800
									//   2 = NOAA SAME RX
									//   3 = ???
									//   4 = FSK 2.4K and FFSK 1200 / 2400
									//   5 = ???
									//   6 = ???
									//   7 = ???
									//
				(1u << 0));			// 1 FSK enable
									//   0 = disable
									//   1 = enable
		break;
	}

	// REG_5A .. bytes 0 & 1 sync pattern
	//
	// <15:8> sync byte 0
	// < 7:0> sync byte 1
	BK4819_WriteRegister(BK4819_REG_5A, 0x3072);

	// REG_5B .. bytes 2 & 3 sync pattern
	//
	// <15:8> sync byte 2
	// < 7:0> sync byte 3
	BK4819_WriteRegister(BK4819_REG_5B, 0x576C);

	// disable CRC
	BK4819_WriteRegister(BK4819_REG_5C, 0x5625);

	// set the almost full threshold
	if(rx)
		BK4819_WriteRegister(BK4819_REG_5E, (64u << 3) | (1u << 0));  // 0 ~ 127, 0 ~ 7

	// packet size .. sync + packet - size of a single packet

	uint16_t size = sizeof(dataPacket.serializedArray);
	// size -= (fsk_reg59 & (1u << 3)) ? 4 : 2;
	if(rx)
		size = (((size + 1) / 2) * 2) + 2;             // round up to even, else FSK RX doesn't work

	BK4819_WriteRegister(BK4819_REG_5D, (size << 8));
	// BK4819_WriteRegister(BK4819_REG_5D, ((sizeof(dataPacket.serializedArray)) << 8));

	// clear FIFO's
	BK4819_FskClearFifo();

	// configure main FSK params
	BK4819_WriteRegister(BK4819_REG_59,
				(0u        <<       15) |   // 0/1     1 = clear TX FIFO
				(0u        <<       14) |   // 0/1     1 = clear RX FIFO
				(0u        <<       13) |   // 0/1     1 = scramble
				(0u        <<       12) |   // 0/1     1 = enable RX
				(0u        <<       11) |   // 0/1     1 = enable TX
				(0u        <<       10) |   // 0/1     1 = invert data when RX
				(0u        <<        9) |   // 0/1     1 = invert data when TX
				(0u        <<        8) |   // 0/1     ???
				((rx ? 0u : 15u) <<  4) |   // 0 ~ 15  preamble length .. bit toggling
				(1u        <<        3) |   // 0/1     sync length
				(0u        <<        0)     // 0 ~ 7   ???
				
	);

	// clear interupts
	BK4819_WriteRegister(BK4819_REG_02, 0);
}

#endif
