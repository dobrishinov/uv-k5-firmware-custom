/* Original work Copyright 2023 joaquimorg
 * https://github.com/joaquimorg
 *
 * Modified work Copyright 2024 kamilsss655
 * https://github.com/kamilsss655
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

#include <string.h>
#include "app/messenger.h"
#include "driver/st7565.h"
#include "external/printf/printf.h"
#include "misc.h"
#include "settings.h"
#include "ui/messenger.h"
#include "ui/helper.h"
#include "ui/inputbox.h"
#include "ui/ui.h"

#ifdef ENABLE_MESSENGER_KEYBOARD_LETTERS_HINTS
// Helper function to generate T9 display strings
void GenerateT9DisplayString(char *output, size_t outputSize, const char *table, unsigned char count) {
    if (count == 1) {
        // Handle single character (e.g., numbers)
        snprintf(output, outputSize, "( %c )", table[0]);
    //} else if (count == 2) {
        // Handle two characters
        //snprintf(output, outputSize, "( %c %c )", table[0], table[1]);
    } else if (count == 3) {
        // Handle three characters
        snprintf(output, outputSize, "( %c %c %c )", table[0], table[1], table[2]);
    } else if (count == 4) {
        // Handle four characters with ellipsis
        snprintf(output, outputSize, "( %c %c %c %c )", table[0], table[1], table[2], table[3]);
    } else if (count == 5) {
		// Handle four characters with ellipsis
		snprintf(output, outputSize, "( %c %c %c %c %c )", table[0], table[1], table[2], table[3], table[4]);
	}
}

void DisplayT9String(KEY_Code_t key, KeyboardType type) {
    char displayString[14];
    if (key >= KEY_1 && key <= KEY_9) {
        unsigned char index = key - KEY_1;
        if (type == UPPERCASE) {
            GenerateT9DisplayString(displayString, sizeof(displayString), T9TableUp[index], numberOfLettersAssignedToKey[index]);
        } else if (type == LOWERCASE) {
            GenerateT9DisplayString(displayString, sizeof(displayString), T9TableLow[index], numberOfLettersAssignedToKey[index]);
        } else if (type == NUMERIC) {
            GenerateT9DisplayString(displayString, sizeof(displayString), T9TableNum[index], numberOfNumsAssignedToKey[index]);
        }
        GUI_DisplaySmallest(displayString, 21, 39, false, true);
    }
}
#endif

void UI_DisplayMSG(void) {
	
	static char String[37];
	static char CounterString[8];

	memset(gFrameBuffer, 0, sizeof(gFrameBuffer));
	//memset(String, 0, sizeof(String));

	//UI_PrintStringSmallBold("MESSENGER", 0, 127, 0);
	//UI_PrintStringSmall("Messenger", 1, 127, 0);

	//UI_DrawDottedLineBuffer(gFrameBuffer, 2, 3, 26, 3, true, 2);
	//UI_DrawDottedLineBuffer(gFrameBuffer, 100, 3, 126, 3, true, 2);

	/*if ( msgStatus == SENDING ) {
		GUI_DisplaySmallest("SENDING", 100, 6, false, true);
	} else if ( msgStatus == RECEIVING ) {
		GUI_DisplaySmallest("RECEIVING", 100, 6, false, true);
	} else {
		GUI_DisplaySmallest("READY", 100, 6, false, true);
	}*/

	// RX Screen

	//GUI_DisplaySmallest("RX", 4, 34, false, true);

	//memset(String, 0, sizeof(String));
	
	uint8_t startMessageIndex = currentPage * 5; // Start index for the current page
    uint8_t mPos = 1;
    const uint8_t mLine = 7;

    for (int i = 0; i < 5; ++i) {
        if (strlen(rxMessage[startMessageIndex + i]) > 0) {
            GUI_DisplaySmallest(rxMessage[startMessageIndex + i], 0, mPos, false, true);
        }
        mPos += mLine;
    }

	// Display the current page number
    snprintf(CounterString, sizeof(CounterString), "%d/8", currentPage + 1);
    GUI_DisplaySmallest(CounterString, 114, 39, false, true);
	UI_DrawRectangleBuffer(gFrameBuffer, 111, 37, 127, 45, true);

	// TX Screen
	memset(String, 0, sizeof(String));
	switch (keyboardType) {
		case NUMERIC:
			strcpy(String, "123");
			break;
		case UPPERCASE:
			strcpy(String, "ABC");
			break;
		default:
			strcpy(String, "abc");
			break;
	}
	
	if (!msgWaitingForAck)
	{
		switch (keyboardKey) {
		#ifdef ENABLE_MESSENGER_KEYBOARD_LETTERS_HINTS
			case KEY_1:
			case KEY_2:
			case KEY_3:
			case KEY_4:
			case KEY_5:
			case KEY_6:
			case KEY_7:
			case KEY_8:
			case KEY_9:
				DisplayT9String(keyboardKey, keyboardType);
				isMsgReceived = 0;
				hasNewMessage = 0;
				optionsButtonsTextState = 0;
				break;
		#endif
			case KEY_STAR: {
				const char *keyboardTypeStrings[] = {
					"UPPERCASE", // UPPERCASE
					"LOWERCASE", // LOWERCASE
					"NUMERIC"    // NUMERIC
				};
			
				if (keyboardType <= NUMERIC) { // Only check the upper bound
					GUI_DisplaySmallest(keyboardTypeStrings[keyboardType], 21, 39, false, true);
					//UI_DisplayPopup("T9 Keyboard 0");
				}
				isMsgReceived = 0;
				hasNewMessage = 0;
				optionsButtonsTextState = 0;
				break;
			}
			case KEY_0:
				if ( keyboardType != NUMERIC ) {
					GUI_DisplaySmallest("SPACE", 21, 39, false, true);
					//UI_DisplayPopup("T9 Keyboard 0");
				} 
			#ifdef ENABLE_MESSENGER_KEYBOARD_LETTERS_HINTS
				else {
					GUI_DisplaySmallest("( 0 )", 21, 39, false, true);
				}
			#endif
				isMsgReceived = 0;
				hasNewMessage = 0;
				optionsButtonsTextState = 0;
				break;
			case KEY_F:
				GUI_DisplaySmallest("DELETE", 21, 39, false, true);
				//UI_DisplayPopup("T9 Keyboard F");
				isMsgReceived = 0;
				hasNewMessage = 0;
				optionsButtonsTextState = 0;
				break;
			case KEY_MENU:
				// if (!hasNewMessage && PAYLOAD_LENGTH_LIMITED - strlen(cMessage) != PAYLOAD_LENGTH_LIMITED)
				// {
				// 	GUI_DisplaySmallest("SENDING...", 21, 39, false, true);
				// }

				hasNewMessage = 0;
				optionsButtonsTextState = 0;
				currentPage = 7; // Default to the last page
				
				//GUI_DisplaySmallest("", 64, 39, false, true);
				//UI_DisplayPopup("T9 Keyboard MENU");
				//isMsgReceived = 0;
				break;
			default:
				isMsgReceived = 0;
				//hasNewMessage = 0;
				break;
		}

		// Check the flag and display a specific message if it's set
		if (optionsButtonsTextState > 0 && optionsButtonsTextState < 5) {
			const char* message = (optionsButtonsTextState == 1) ? "TEXT PASTED" :
								  (optionsButtonsTextState == 2) ? "TEXT COPIED" :
								  (optionsButtonsTextState == 3) ? "TO: LATEST DM" :
								  "LAST SENT TEXT";
			GUI_DisplaySmallest(message, 21, 39, false, true);
		}
	}

	// if (isMsgReceived && !hasNewMessage)
	// {
	// 	GUI_DisplaySmallest("DELIVERED!", 21, 39, false, true);
	// } 

	// if (msgWaitingForAck && msgRetryCount <= 3) {
	// 	char retryStr[16];
	// 	if (msgRetryCount < 3) {
	// 		snprintf(retryStr, sizeof(retryStr), "(%d/3)SENDING...", msgRetryCount);
	// 		GUI_DisplaySmallest(retryStr, 21, 39, false, true);
	// 	} else {
	// 		GUI_DisplaySmallest("(!)", 21, 39, false, true);
	// 	}
	// }

	if (isMsgReceived && !hasNewMessage) {
		GUI_DisplaySmallest("DELIVERED!", 21, 39, false, true);
	} else if (msgWaitingForAck && msgRetryCount <= 3) {
		char retryStr[20];
		if (msgAutoRetryEnabled) {
			if (msgRetryCount < 3) {
				snprintf(retryStr, sizeof(retryStr), "(%d/3)SENDING...", msgRetryCount);
				GUI_DisplaySmallest(retryStr, 21, 39, false, true);
				msgAutoRetryPopup = 0;
			} else {
				GUI_DisplaySmallest("(3/3)", 21, 39, false, true);
			}
		} else {
			// Auto-retry is OFF: only show SENDING... for the first attempt, then show (!) if not delivered
			if (msgRetryCount == 1 && msgWaitingForAck) {
				GUI_DisplaySmallest("SENDING...", 21, 39, false, true);
				msgAutoRetryPopup = 0;
				msgWaitingForAck = false;
			}
			// } else {
			// 	GUI_DisplaySmallest("(!)", 21, 39, false, true);
			// }
		}
	}

	// if (msgAutoRetryPopup) {
	// 	switch (msgAutoRetryPopup) {
	// 		case 1:
	// 			GUI_DisplaySmallest("Repeater ON", 21, 39, false, true);
	// 			break;
	// 		case 2:
	// 			GUI_DisplaySmallest("Repeater OFF", 21, 39, false, true);
	// 			break;
	// 		case 3:
	// 			GUI_DisplaySmallest("Auto Retry ON", 21, 39, false, true);
	// 			break;
	// 		case 4:
	// 			GUI_DisplaySmallest("Auto Retry OFF", 21, 39, false, true);
	// 			break;
	// 		default:
	// 			GUI_DisplaySmallest("Unknown State", 21, 39, false, true);
	// 			break;
	// 	}
	// 	msgAutoRetryPopup = 0; // Reset after displaying once
	// }

	if (msgAutoRetryPopup) {
		switch (msgAutoRetryPopup) {
			case 1:
				GUI_DisplaySmallest("MODE: AUTO RETRY", 21, 39, false, true);
				break;
			case 2:
				GUI_DisplaySmallest("MODE: REPEATER", 21, 39, false, true);
				break;
			case 3:
				GUI_DisplaySmallest("MODE: NORMAL", 21, 39, false, true);
				break;
			default:
				GUI_DisplaySmallest("Unknown Mode", 21, 39, false, true);
				break;
		}
		msgAutoRetryPopup = 0; // Reset after displaying once
	}
	
	// else if (hasNewMessage && !isMsgReceived) {
	// 	hasNewMessage = 1;
	// 	GUI_DisplaySmallest("(NEW MESSAGE)", 21, 39, false, true);
	// }

	// if (hasNewMessage && PAYLOAD_LENGTH_LIMITED - strlen(cMessage) == PAYLOAD_LENGTH_LIMITED) {
	// 	isMsgReceived = 0;
	// 	GUI_DisplaySmallest("(NEW MESSAGE)", 21, 39, false, true);
	// }

	// Vertical Line to separate Message Status (Sending/Receiving/Received) and Message
	//UI_DrawLineBuffer(gFrameBuffer, 5, 36, 5, 0, true);
	UI_DrawDottedLineBuffer(gFrameBuffer, 5, 34, 5, 0, true, 2);
	
	// Rectangle for T9 Keyboard selection
	UI_DrawRectangleBuffer(gFrameBuffer, 1, 37, 17, 45, true);
	GUI_DisplaySmallest(String, 4, 39, false, true);

	// Display the remaining words counter
    memset(CounterString, 0, sizeof(CounterString));
    sprintf(CounterString, "%d/%d", PAYLOAD_LENGTH_LIMITED - strlen(cMessage), PAYLOAD_LENGTH_LIMITED);
    GUI_DisplaySmallest(CounterString, PAYLOAD_LENGTH_LIMITED - strlen(cMessage) < 10 ? 93 : 89, 39, false, true); // Adjust position as needed

	// 1st Horizontal Dot Line before input text
	UI_DrawDottedLineBuffer(gFrameBuffer, 21, 45, 108, 45, true, 2);
	// 2nd Horizontal Dot Line before input text
	UI_DrawDottedLineBuffer(gFrameBuffer, 21, 37, 108, 37, true, 2);
    // Display current message
	memset(String, 0, sizeof(String));
	sprintf(String, "%s_", cMessage);
	//UI_PrintStringSmall(String, 3, 0, 6);
	GUI_DisplaySmallest(String, 2, 48, false, true);
	// 3rd Horizontal Dot Line after input text
	UI_DrawDottedLineBuffer(gFrameBuffer, 1, 55, 128, 55, true, 2);

	//UI_DrawRectangleBuffer(gFrameBuffer, 0, 46, 128, 54, true);


	// debug msg
	/*memset(String, 0, sizeof(String));
	sprintf(String, "S:%u", gErrorsDuringMSG);
	GUI_DisplaySmallest(String, 4, 12, false, true);

	memset(String, 0, sizeof(String));
	for (uint8_t i = 0; i < 19; i++) {
		sprintf(&String[i*2], "%02X", rxMessage[i]);
	}
	
	GUI_DisplaySmallest(String, 20, 34, false, true);*/

	ST7565_BlitFullScreen();
}

#endif