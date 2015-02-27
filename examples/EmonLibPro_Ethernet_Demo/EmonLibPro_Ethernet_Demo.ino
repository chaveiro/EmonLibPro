/*
    Energy monitor Library Pro
    Based on emonTx hardware from OpenEnergyMonitor http://openenergymonitor.org/emon/
    This is a generic library for any voltage and current sensors.
    Interrupt based, implements a zero cross detector and phase-locked loop for better precision.
    Reports Voltage, Frequency, Current, Active Power, Aparent Power and Power Factor.
    Completely line frequency independent.
    This library idea cames from ATMEL AVR465: Single-Phase Power/Energy Meter.
    
    Get latest version at https://github.com/chaveiro/EmonLibPro
    
    Copyright (C) 2013-2015  Nuno Chaveiro  nchaveiro[at]gmail.com  Lisbon, Portugal

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
 
#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include "EmonLibPro.h"
//--------------------------------------------------------------------------------------------------
//Ethernet Vars
#define CMS_HOST "emon.server.com"                  // Host name
//#define CMS_HOST "192.168.1.250"                    // Host name
#define CMS_PORT 80                                   // Port
#define CMS_NODEID_PREFIX "0"                         // EmonCMS NodeId (x part on xY)
#define CMS_APIKEY "fc9134ff558200d1fdd3557094d1b762" // EmonCMS API KEY

// MAC address for your controller (See sticker on the shield)
byte mac[] = { 0x90, 0xA2, 0xDA, 0x00, 0x35, 0xA0 }; 

// If you don't want to use DNS (and reduce your sketch size) use numeric IP
//IPAddress server(74,125,232,128); // numeric IP (no DNS)
char server[] = CMS_HOST;           // name address (using DNS)
IPAddress ip(192,168,1,199);        // Static IP address to use if the DHCP fails to assign
EthernetServer ethServer(80);       // Init Ethernet server library
EthernetClient client;              // Init Ethernet client with IP address and port of the server

const int kNetworkTimeout = 1750;   // Wait timeout for receiving data
const int kNetworkDelay = 50;       // Wait if no data is available before trying again

EthernetClient srvClient;


//--------------------------------------------------------------------------------------------------
//Emon Vars
int userCommand;
EmonLibPro Emon;


void setup()
{
  Serial.begin(9600);
  Serial.print("EMON: v");
  Serial.println(EMONLIBPROVERSION);
  
  Emon = EmonLibPro();    // Required
  Emon.begin();           // Required
  
  ethInit();

  delay(1000);  
  Emon.calculateResult();
  delay(1000); 

  printMenu();
  userCommand = 5;
}

void loop()
{
  // listen for incoming clients
  srvClient = ethServer.available();

  if(srvClient && Emon.FlagCYCLE_FULL && EmonLibPro::FlagCYCLE_DIAG) {
      serverEth();
      EmonLibPro::FlagCYCLE_DIAG = false;  //Must reset after read to know next batch
      //Emon.FlagCYCLE_FULL = false;  //Must reset after read to know next batch.
  }else if(srvClient && !EmonLibPro::FlagCYCLE_DIAG) {
      EmonLibPro::FlagCYCLE_DIAG = true;  //Must reset after read to know next batch.
  }

  
  if(userCommand == 1 && Emon.FlagCALC_READY) { 
      Emon.calculateResult();
      for (byte i=0;i<CURRENTCOUNT;i++){
        printResults(i);
      }
  }
  
  if(userCommand == 2 && Emon.FlagCYCLE_FULL) {
      for (byte i=0;i<VOLTSCOUNT + CURRENTCOUNT;i++){
        Serial.print("ADC");
        Serial.print(0+i);
        Serial.print(",");
        serialDataTable(i);
      }
      Emon.FlagCYCLE_FULL = false;  //Must reset after read to know next batch.
  }
  
  if(userCommand == 3 && Emon.FlagCALC_READY) {
        Emon.printStatus();
        userCommand = 0;
  }  

  if (userCommand == 4 && EmonLibPro::FlagOutOfTime) { // This should never be true.  Reduce sampling rate if it is!
	Serial.println("$");         // Alarm that previous ADC processing has run out of time before timer event. Must decrease SAMPLESPSEC
  }

  if(userCommand == 5 && Emon.FlagCALC_READY) {
        Emon.calculateResult();
        postEth();
  }

  ethDhcpMaintain();      // Maintain dhcp refresh
  
  if(Serial.available())
  {
    userCommand=Serial.parseInt();
    Serial.print("Got: ");
    Serial.println(userCommand);
  }
  
}


// #########################################################################
// ## Ethernet support
// #########################################################################
void ethInit(){
    // start the Ethernet connection:
  if (Ethernet.begin(mac) == 0) {
    Serial.println("ETH: DHCP failed");
    // no point in carrying on, so do nothing forevermore:
    // try to congifure using IP address instead of DHCP:
    Ethernet.begin(mac, ip);
  } else { 
      Serial.println("ETH: DHCP ok");
  }
  // give the Ethernet shield a second to initialize:
  delay(500);

  ethServer.begin();
  Serial.println("ETH: Server started.");
  ethIpStatus();
}

// Print your local IP address:
void ethIpStatus(){
  Serial.print("ETH: IP=");
  for (byte thisByte = 0; thisByte < 4; thisByte++) {
    // print the value of each byte of the IP address:
    Serial.print(Ethernet.localIP()[thisByte], DEC);
    Serial.print(".");
  }
  Serial.println();
}

// Mantain DHCP leases
void ethDhcpMaintain(){
  int rc = Ethernet.maintain();    
  switch ( rc ){
    case DHCP_CHECK_NONE:
      //nothing done
      break;
    case DHCP_CHECK_RENEW_FAIL:
    case DHCP_CHECK_REBIND_FAIL:
      Serial.print("ETH: DHCP renew fail");
      ethIpStatus();
      break;
    case DHCP_CHECK_RENEW_OK:
    case DHCP_CHECK_REBIND_OK:
      Serial.print("ETH: DHCP renewed");
      ethIpStatus();
      break;
    default:
      Serial.print("ETH: DHCP other"); //this is actually a error, it will retry though
      break;
  }
}

// Awnsers back to a connecting client the current cycle vars
void serverEth() {
  //  Serial.print("ETH: Server...");
  boolean currentLineIsBlank = true;
  unsigned long timeoutStart = millis();
  char c;
  while ((srvClient.connected() || srvClient.available()) &&
         (millis() - timeoutStart < (int)kNetworkTimeout)) {
    if (srvClient.available()) {
  	  c = srvClient.read();
  	  //Serial.write(c);
  	  // If you've gotten to the end of the line (received a newline character) and the line is blank,
          // the http request has ended, so you can send a reply
  	  if (c == '\n' && currentLineIsBlank) {
  		// send a standard http response header
  		srvClient.println("HTTP/1.1 200 OK");
  		//srvClient.println("Content-Type: application/json");
  		//srvClient.println("Connection: close");  // the connection will be closed after completion of the response
  		srvClient.println("Access-Control-Allow-Origin: *");
  		//srvClient.println("Refresh: 5");  // refresh the page automatically every 5 sec
  		srvClient.println();
  		//srvClient.println("<!DOCTYPE HTML>");
  		//srvClient.println("<html>");
  		ethDataTable();
  		//srvClient.println("</html>");
  		break;
  	  }
  	  if (c == '\n') {
  		currentLineIsBlank = true;  // you're starting a new line
  	  }
  	  else if (c != '\r') {
  		currentLineIsBlank = false;  // you've gotten a character on the current line
  	  }
    timeoutStart = millis();  //reset the timeout counter
    } else {
          delay(kNetworkDelay);  // We haven't got any data, so let's pause to allow some to arrive
    }
  }
  srvClient.stop();
}

// This is part of the answer to a connecting client
void ethDataTable(){
  byte i,x;
  int sizearr = (Emon.SamplesPerCycleTotal/Emon.CyclesPerTotal);
  if (sizearr > CYCLEARRSIZE) sizearr = CYCLEARRSIZE;
  
  srvClient.print("[[");
  for (i=0;i<VOLTSCOUNT + CURRENTCOUNT;i++){
    if (i>0) srvClient.print(",");
    srvClient.print("\"ADC");
    srvClient.print(0+i);
    srvClient.print("\"");
  }
  srvClient.print("],[");
  for (x=0;x<=(sizearr)-1;x++){
    if (x>0) srvClient.print(",[");
      for (i=0;i<VOLTSCOUNT + CURRENTCOUNT;i++){
      if (i>0) srvClient.print(",");
      srvClient.print(Emon.Sample[i].CycleArr[x]);
    }
    srvClient.print("]");
  }
  srvClient.print("]");
}

// Post to server the current result
void postEth()
{
  byte i;
//  Serial.print("ETH: Post...");
  if (client.connect(server, CMS_PORT)) {
    // Make a HTTP request:
    //http://emon.server.com/input/bulk.json?data=[[0,1,10,20,30,40,50,60]]
    client.print("GET /input/bulk.json?apikey=" CMS_APIKEY "&data=[");
    for (i=0;i<CURRENTCOUNT;i++){
      if (i>0) {
        client.print(",");
      }
      client.print("[0,");
      client.print(CMS_NODEID_PREFIX);
      client.print(i);
      client.print(",");
      client.print(Emon.ResultV[0].U);
      client.print(",");
      client.print(Emon.ResultV[0].HZ);
      client.print(",");
      client.print(Emon.ResultP[i].I);
      client.print(",");
      client.print(Emon.ResultP[i].P);
      client.print(",");
      client.print(Emon.ResultP[i].S);
      client.print(",");
      client.print(Emon.ResultP[i].F);
#ifdef USEPLL
      client.print(",");
      client.print(Emon.pllUnlocked);
#endif
      client.print("]");
    }
    client.print("]");
    client.println(" HTTP/1.0");
    client.println("Host: " CMS_HOST);
    client.println("User-Agent: EmonLibPro");
    client.println("Connection: close");
    client.println();
    //Serial.print("ETH: Post");
  
    // Now we've got to the response
    unsigned long timeoutStart = millis();
    char c;
    // Whilst we haven't timed out & haven't reached the end of the body
    while ((client.connected() || client.available()) &&
           (millis() - timeoutStart < (int)kNetworkTimeout))
    {
        if (client.available())
        {
            c = client.read();
            // Print out this character
            //Serial.print(c);
            timeoutStart = millis();  //reset the timeout counter
        }
        else
        {
            delay(kNetworkDelay);  // We haven't got any data, so let's pause to allow some to arrive
        }
    }
  } else {
    Serial.print("ETH: Post Fail");    // didn't get a connection to the server:
  }
  client.stop();
}



// #########################################################################
// ## Serial support
// #########################################################################
void printMenu(){
/*    Serial.println("EmonLipPro Demo");
    Serial.println(" 1 - Print cycle data. (internal var data for each cycle)");
    Serial.println(" 2 - Print Calculated data. Change interval in define CALCULATECYCLES.");
    Serial.println(" 3 - Print Lib Status.");
    Serial.println(" 4 - Check if sample rate is acceptable. Should not see the $ char.");
    Serial.println(" 5 - Post to server.");
*/    Serial.println("Press a key...");
}


// Print calculated results
void printResults(byte i)
{
    Serial.print("Result");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(Emon.ResultV[0].U);
    Serial.print("VAC\t");
    Serial.print(Emon.ResultV[0].HZ);
    Serial.print("Hz\t");
    Serial.print(Emon.ResultP[i].I);
    Serial.print("A\t");
    Serial.print(Emon.ResultP[i].P);
    Serial.print("W\t");
    Serial.print(Emon.ResultP[i].S);
    Serial.print("VA\t");
    Serial.print(Emon.ResultP[i].F);
    Serial.print("Pfact");
#ifdef USEPLL
    if(!Emon.pllUnlocked) Serial.print(" L");
#endif
    Serial.println("\t");
}


// Exposes the internal sampled values for all samples on a cycle
// Usefull to display the signal on a graph.
void serialDataTable(byte b){
  int sizearr = (Emon.SamplesPerCycleTotal/Emon.CyclesPerTotal);
  if (sizearr > CYCLEARRSIZE) sizearr = CYCLEARRSIZE;
  for (byte i=0;i<=sizearr;i++){
    Serial.print(Emon.Sample[b].CycleArr[i]);
    Serial.print(",");
  }
  Serial.println("\t");
}