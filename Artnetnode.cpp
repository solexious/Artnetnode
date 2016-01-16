/*

Copyright (c) Charles Yarnold charlesyarnold@gmail.com 2015

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, under version 2 of the License.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <Artnetnode.h>
#include "user_interface.h"
// #include <HardwareSerial.cpp>

Artnetnode::Artnetnode() {
  // Ethernet.enableLinkLed();
  // Ethernet.enableActivityLed();

  // Initalise DMXOutput array
  for(int i = 0; i < DMX_MAX_OUTPUTS; i++){
    DMXOutputs[i][0] = -1;
    DMXOutputs[i][1] = -1;
    DMXOutputs[i][2] = 0;
  }

  // Start DMX tick clock
  msSinceDMXSend = 0;

  // Init DMX buffers
  for(int i = 0; i < DMX_MAX_OUTPUTS; i++){
    memset(DMXBuffer[i], 0, sizeof(DMXBuffer[i]));
  }
}

uint8_t Artnetnode::begin(char ssid[], char pass[], uint8_t numOutputs){
  if(WiFi.begin(ssid, pass)){
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
  
    Udp.begin(ARTNET_PORT);
    localIP = WiFi.localIP();
    localMask = WiFi.subnetMask();
    localBroadcast = IPAddress(localIP | ~localMask);

    uint8_t mac[6];
    WiFi.macAddress(mac);

    PollReplyPacket.setMac(mac);
    PollReplyPacket.setIP(localIP);
    PollReplyPacket.canDHCP(true);
    PollReplyPacket.isDHCP(true);

    return 1;
  }
  else{
    return 0;
  }
}

uint8_t Artnetnode::setShortName(char name[]){
  PollReplyPacket.setShortName(name);
}

uint8_t Artnetnode::setLongName(char name[]){
  PollReplyPacket.setLongName(name);
}

uint8_t Artnetnode::setName(char name[]){
  PollReplyPacket.setShortName(name);
  PollReplyPacket.setLongName(name);
}

uint8_t Artnetnode::setStartingUniverse(uint16_t startingUniverse){
  PollReplyPacket.setStartingUniverse(startingUniverse);
}

uint16_t Artnetnode::read(){
  packetSize = Udp.parsePacket();

  
  if (packetSize <= ARTNET_MAX_BUFFER && packetSize > 0){ 
    Udp.read(artnetPacket, ARTNET_MAX_BUFFER);

    // Check packetID equals "Art-Net"
    for (int i = 0 ; i < 9 ; i++){
      if (artnetPacket[i] != ARTNET_ID[i])
        return 0;
    }
      
    opcode = artnetPacket[8] | artnetPacket[9] << 8; 

    if (opcode == OpDmx){
      return handleDMX();
    }
    else if (opcode == OpPoll){
    	return handlePollRequest();
    }
  }
  else{
    return 0;
  }
}

bool Artnetnode::isBroadcast(){
  if(Udp.destinationIP() == localBroadcast){
    return true;
  }
  else{
    return false;
  }
}

uint16_t Artnetnode::handleDMX(){
  if(isBroadcast()){
    return 0;
  }
  else{
    // Get universe
    uint16_t universe = artnetPacket[14] | artnetPacket[15] << 8;

    // Get DMX frame length
    uint16_t dmxDataLength = artnetPacket[17] | artnetPacket[16] << 8;


    for(int a = 0; a < DMX_MAX_OUTPUTS; a++){
      if(DMXOutputs[a][1] == universe){
        for (int i = 0 ; i < DMX_MAX_BUFFER ; i++){
          if(i < dmxDataLength){
            DMXBuffer[a][i] = artnetPacket[i+ARTNET_DMX_START_LOC];
          }
          else{
            DMXBuffer[a][i] = 0;
          }
        }
      }
    }

    return OpDmx;
  }
}

uint8_t Artnetnode::returnDMXValue(uint8_t outputID, uint8_t channel){
  return DMXBuffer[outputID][channel];
}

uint16_t Artnetnode::handlePollRequest(){
  if(isBroadcast()){
    
    Udp.beginPacket(localBroadcast, ARTNET_PORT);
    Udp.write(PollReplyPacket.printPacket(), sizeof(PollReplyPacket.packet));
    Udp.endPacket();

    return OpPoll;
  }
  else{
    return 0;
  }
}

void Artnetnode::enableDMX(){
  DMXOutputStatus = true;
}

void Artnetnode::disableDMX(){
  DMXOutputStatus = false;
}

void Artnetnode::enableDMXOutput(uint8_t outputID){
  DMXOutputs[outputID][2] = 1;

  int numEnabled = 0;
  for(int i = 0; i < DMX_MAX_OUTPUTS; i++){
    if(DMXOutputs[i][2]==1){
      if(numEnabled<4){
        numEnabled++;
      }
    }
  }
  PollReplyPacket.setNumPorts(numEnabled);
  PollReplyPacket.setOutputEnabled(outputID);
}

void Artnetnode::disableDMXOutput(uint8_t outputID){
  DMXOutputs[outputID][2] = 0;

  int numEnabled = 0;
  for(int i = 0; i < DMX_MAX_OUTPUTS; i++){
    if(DMXOutputs[i][2]==1){
      if(numEnabled<4){
        numEnabled++;
      }
    }
  }
  PollReplyPacket.setNumPorts(numEnabled);
  PollReplyPacket.setOutputDisabled(outputID);
}

uint8_t Artnetnode::setDMXOutput(uint8_t outputID, uint8_t uartNum, uint16_t attachedUniverse){
  // Validate input
  if(outputID>-1 && uartNum>-1 && attachedUniverse>-1){
    DMXOutputs[outputID][0] = uartNum;
    DMXOutputs[outputID][1] = attachedUniverse;
    DMXOutputs[outputID][2] = 0;
    PollReplyPacket.setSwOut(outputID, attachedUniverse);
    return 1;
  }
  else{
    return 0;
  }
}

void Artnetnode::tickDMX(uint32_t time){
	msSinceDMXSend += time;
  if(msSinceDMXSend > DMX_MS_BETWEEN_TICKS){
    sendDMX();
    msSinceDMXSend = 0;
  }
}

void Artnetnode::sendDMX(){
	if(DMXOutputStatus){
    for(int i = 0; i <= DMX_MAX_OUTPUTS; i++){
      if(DMXOutputs[i][2]==1){
        uartDMX(i, DMXOutputs[i][0]);
      }
    }
	}
}

void Artnetnode::uartDMX(uint8_t outputID, uint8_t uartNum){
  if(uartNum == 1){
    // Serial1.begin(125000, SERIAL_8N2);
    // Serial1.write((uint8_t)0);
    // Serial1.flush();

    /*Serial.begin(125000);
    ROM_UARTConfigSetExpClk(UART0_BASE, F_CPU, 125000,
                              (UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_TWO |
                              UART_CONFIG_WLEN_8));
    Serial.write((uint8_t)0);
    Serial.flush();*/

    // Serial1.begin(250000, SERIAL_8N2);
    // Serial1.write(DMXBuffer[outputID], 512);

    /*Serial.begin(250000);
    ROM_UARTConfigSetExpClk(UART0_BASE, F_CPU, 250000,
                              (UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_TWO |
                              UART_CONFIG_WLEN_8));
    Serial.write((uint8_t)0);
    Serial.write(DMXBuffer[outputID], 512);*/

    digitalWrite(2, HIGH);
    Serial1.begin(83333, SERIAL_8N1);
    Serial1.write(0);
    Serial1.flush();
    delay(1);
    Serial1.end();

    //send data
    Serial1.begin(250000, SERIAL_8N2);
    digitalWrite(2, LOW);
    Serial1.write(DMXBuffer[outputID], 512);
    Serial1.flush();
    delay(1);
    Serial1.end();

    Serial.println(DMXBuffer[outputID][0]);

  }
}