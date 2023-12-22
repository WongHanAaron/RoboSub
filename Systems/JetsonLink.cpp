#include "JetsonLink.hpp"
#include <stdio.h>
float Exodus::charToFloat(char * charArray) {
    Exodus::floatConvert fValue;
    fValue.l = ((long) charArray[0] << 24)| // The first send char is the MSB
               ((long) charArray[1] << 16)|
               ((long) charArray[2] << 8 )|
               ((long) charArray[3] );      // The last char is the LSB
    return fValue.f;
}

void Exodus::charToBitStream(char * charArray, char* returnedStream) {
    if ((charArray[3]      & 0x01) == 0x01) returnedStream[31] = '1'; else returnedStream[31] = '0';
    if ((charArray[3] >> 1 & 0x01) == 0x01) returnedStream[30] = '1'; else returnedStream[30] = '0';
    if ((charArray[3] >> 2 & 0x01) == 0x01) returnedStream[29] = '1'; else returnedStream[29] = '0';
    if ((charArray[3] >> 3 & 0x01) == 0x01) returnedStream[28] = '1'; else returnedStream[28] = '0';
    if ((charArray[3] >> 4 & 0x01) == 0x01) returnedStream[27] = '1'; else returnedStream[27] = '0';
    if ((charArray[3] >> 5 & 0x01) == 0x01) returnedStream[26] = '1'; else returnedStream[26] = '0';
    if ((charArray[3] >> 6 & 0x01) == 0x01) returnedStream[25] = '1'; else returnedStream[25] = '0';
    if ((charArray[3] >> 7 & 0x01) == 0x01) returnedStream[24] = '1'; else returnedStream[24] = '0';
    if ((charArray[2]      & 0x01) == 0x01) returnedStream[23] = '1'; else returnedStream[23] = '0';
    if ((charArray[2] >> 1 & 0x01) == 0x01) returnedStream[22] = '1'; else returnedStream[22] = '0';
    if ((charArray[2] >> 2 & 0x01) == 0x01) returnedStream[21] = '1'; else returnedStream[21] = '0';
    if ((charArray[2] >> 3 & 0x01) == 0x01) returnedStream[20] = '1'; else returnedStream[20] = '0';
    if ((charArray[2] >> 4 & 0x01) == 0x01) returnedStream[19] = '1'; else returnedStream[19] = '0';
    if ((charArray[2] >> 5 & 0x01) == 0x01) returnedStream[18] = '1'; else returnedStream[18] = '0';
    if ((charArray[2] >> 6 & 0x01) == 0x01) returnedStream[17] = '1'; else returnedStream[17] = '0';
    if ((charArray[2] >> 7 & 0x01) == 0x01) returnedStream[16] = '1'; else returnedStream[16] = '0';
    if ((charArray[1]      & 0x01) == 0x01) returnedStream[15] = '1'; else returnedStream[15] = '0';
    if ((charArray[1] >> 1 & 0x01) == 0x01) returnedStream[14] = '1'; else returnedStream[14] = '0';
    if ((charArray[1] >> 2 & 0x01) == 0x01) returnedStream[13] = '1'; else returnedStream[13] = '0';
    if ((charArray[1] >> 3 & 0x01) == 0x01) returnedStream[12] = '1'; else returnedStream[12] = '0';
    if ((charArray[1] >> 4 & 0x01) == 0x01) returnedStream[11] = '1'; else returnedStream[11] = '0';
    if ((charArray[1] >> 5 & 0x01) == 0x01) returnedStream[10] = '1'; else returnedStream[10] = '0';
    if ((charArray[1] >> 6 & 0x01) == 0x01) returnedStream[9] = '1';  else returnedStream[9] = '0';
    if ((charArray[1] >> 7 & 0x01) == 0x01) returnedStream[8] = '1';  else returnedStream[8] = '0';
    if ((charArray[0]      & 0x01) == 0x01) returnedStream[7] = '1';  else returnedStream[7] = '0';
    if ((charArray[0] >> 1 & 0x01) == 0x01) returnedStream[6] = '1';  else returnedStream[6] = '0';
    if ((charArray[0] >> 2 & 0x01) == 0x01) returnedStream[5] = '1';  else returnedStream[5] = '0';
    if ((charArray[0] >> 3 & 0x01) == 0x01) returnedStream[4] = '1';  else returnedStream[4] = '0';
    if ((charArray[0] >> 4 & 0x01) == 0x01) returnedStream[3] = '1';  else returnedStream[3] = '0';
    if ((charArray[0] >> 5 & 0x01) == 0x01) returnedStream[2] = '1';  else returnedStream[2] = '0';
    if ((charArray[0] >> 6 & 0x01) == 0x01) returnedStream[1] = '1';  else returnedStream[1] = '0';
    if ((charArray[0] >> 7 & 0x01) == 0x01) returnedStream[0] = '1';  else returnedStream[0] = '0';
}

Exodus::JetsonLink::JetsonLink() {}

void Exodus::JetsonLink::sendFloat(float value) {
    if (!initialized) return;
    floatConvert fValue;
    fValue.f = value;
    char sentArray[4];
    sentArray[0] = (fValue.l >> 24) & 0xFF;
    sentArray[1] = (fValue.l >> 16) & 0xFF;
    sentArray[2] = (fValue.l >> 8 ) & 0xFF;
    sentArray[3] = (fValue.l ) & 0xFF;
    write(sentArray, 4);
}

void Exodus::JetsonLink::sendFloat(JetsonLinkProtocol identifier, float value, const char* EoT) {
    if (!initialized) return;
    write((char*)&identifier, 1);
    Exodus::JetsonLink::sendFloat(value);
    printf(EoT);
}

void Exodus::JetsonLink::resetBuffer() {
    for (int i = 0; i < 50; i++) {
        input[i] = '\0';
    }
    foundCommand = false;
    inputIndex = 0;
}


void Exodus::JetsonLink::rightShiftBuffer(int count){
    for (int i = 0; i < 50 - count && input[i] != '\0'; i++) {
        input[i] = input[i + count];
    }
}

void Exodus::JetsonLink::resetInterrupt() {
    // Save our current interrupt state before resetting
    // Clears the current interrupt states
    UARTIntClear(moduleBase, UARTIntStatus(moduleBase, true));
}
