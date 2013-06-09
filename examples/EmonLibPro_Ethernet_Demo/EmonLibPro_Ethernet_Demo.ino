/*
    Energy monitor Library Pro
    Based on emonTx hardware from OpenEnergyMonitor http://openenergymonitor.org/emon/
    This is a generic library for any voltage and current sensors.
    Interrupt based, implements a zero cross detector and phase-locked loop for better precision.
    Completely line frequency independent.
    
    Copyright (C) 2013  Nuno Chaveiro  nchaveiro[at]gmail.com  Lisbon, Portugal

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
#define CMS_NODEID_PREFIX "1"                         // EmonCMS NodeId (x part on xY)
#define CMS_APIKEY "fc9114ff558200d1fdd3f57194d8b712" // EmonCMS API KEY

// Enter a MAC address for your controller below.
// Newer Ethernet shields have a MAC address printed on a sticker on the shield
byte mac[] = { 0x90, 0xA2, 0xDA, 0x00, 0x35, 0xA5 };

// if you don't want to use DNS (and reduce your sketch size)
// use the numeric IP instead of the name for the server:
//IPAddress server(74,125,232,128);  // numeric IP for Google (no DNS)
char server[] = CMS_HOST;    // name address for Google (using DNS)

// Set the static IP address to use if the DHCP fails to assign
IPAddress ip(192,168,1,199);

// Initialize the Ethernet client library
// with the IP address and port of the server
// that you want to connect to (port 80 is default for HTTP):
EthernetClient client;

// Number of milliseconds to wait without receiving any data before we give up
const int kNetworkTimeout = 1750;
// Number of milliseconds to wait if no data is available before trying again
const int kNetworkDelay = 50;

//--------------------------------------------------------------------------------------------------
//Emon Vars
byte i;
int userCommand;

void printCycle(byte i);
void printResults(byte i);
void printMenu();
void printStatus();
void postEth();

EmonLibPro  Emon ;

void setup()
{
  Serial.begin(230400);
  Serial.println("Ready");
  
  Emon = EmonLibPro();    // Required.  
  Emon.begin();
  
  ethInit();

  delay(1000);  
  Emon.calculateResult();
  delay(1000); 

  printMenu();
  userCommand = 4;
}

void loop()
{
  if(Emon.FlagCYCLE_FULL && userCommand == 1) {
      for (i=0;i<CURRENTCOUNT;i++){
          printCycle(i);
      }
      Emon.FlagCYCLE_FULL = false;  //Must reset after read to know next batch.
  }
  if(Emon.FlagCALC_READY && userCommand == 2) { 
      Emon.calculateResult();
      for (i=0;i<CURRENTCOUNT;i++){
        printResults(i);
      }
  }
  if(userCommand == 3) {
        printStatus();
        userCommand = 0;
  }  
  
  if(Emon.FlagCALC_READY && userCommand == 4) {
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
  delay(1000);
}

void ethIpStatus(){
    // print your local IP address:
  Serial.print("ETH: IP=");
  for (byte thisByte = 0; thisByte < 4; thisByte++) {
    // print the value of each byte of the IP address:
    Serial.print(Ethernet.localIP()[thisByte], DEC);
    Serial.print(".");
  }
  Serial.println();
}

void ethDhcpMaintain(){
/*
    returns:
    0/DHCP_CHECK_NONE: nothing happened
    1/DHCP_CHECK_RENEW_FAIL: renew failed
    2/DHCP_CHECK_RENEW_OK: renew success
    3/DHCP_CHECK_REBIND_FAIL: rebind fail
    4/DHCP_CHECK_REBIND_OK: rebind success
*/
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
      Serial.print("ETH: DHCP other");
      //this is actually a error, it will retry though
      break;
  }
}


void printMenu(){
    Serial.println("EmonLipPro Demo");
    Serial.println(" 1 - Print cycle data. (internal vars data for each cycle)");
    Serial.println(" 2 - Print Calculated data. Change interval in define CALCULATESAMPLES.");
    Serial.println(" 3 - Print Lib Status.");
    Serial.println(" 4 - Post to server.");
    Serial.println("Press a key...");
}

void printCycle(byte i)
{
  Serial.print("Cycle");
    Serial.print(i);
    Serial.print(": ");
	Serial.print(Emon.CycleV[i].U2);
	Serial.print(" ");
	Serial.print(Emon.CycleV[i].cHZ);
	Serial.print(" ");
	Serial.print(Emon.CycleP[i].I2);
	Serial.print(" ");
	Serial.print(Emon.CycleP[i].P);
    if(!Emon.pllUnlocked) Serial.print(" L");    
	Serial.println("\t");
}    

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
    Serial.println("Pfact");
}


void postEth()
{
  Serial.print("ETH: Post...");

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
      client.print(",");
      client.print(Emon.pllUnlocked);
      client.print("]");
    }
    client.print("]");
    client.println(" HTTP/1.0");
    client.println("Host: " CMS_HOST);
    client.println("User-Agent: EmonLibPro");
    client.println("Connection: close");
    client.println();
    Serial.print("Sent");
  
    // Now we've got to the response
    unsigned long timeoutStart = millis();
    char c;
    // Whilst we haven't timed out & haven't reached the end of the body
    while ((client.connected() || client.available()) &&
           (millis() - timeoutStart < kNetworkTimeout))
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
            // We haven't got any data, so let's pause to allow some to arrive
            delay(kNetworkDelay);
        }
    }
  } else {
    Serial.print("Fail");    // didn't get a connection to the server:
  }
  client.stop();
}


void printStatus()
{
  if (Emon.FlagOutOfTime) Serial.println("ERROR: Timer has overflown, reduce SAMPLESPSEC.");
  if(Emon.pllUnlocked) { 
      Serial.println("PLL: Unlocked");
  } else {
      Serial.println("PLL: Locked");
  } 
  Serial.print("ADC samples per Cycle: ");
  Serial.println((Emon.SamplesPerCycleTotal/Emon.CyclesPerTotal) * (VOLTSCOUNT + CURRENTCOUNT));
  Serial.print("VOLTSCOUNT: ");
  Serial.println(VOLTSCOUNT);
  Serial.print("CURRENTCOUNT: ");
  Serial.println(CURRENTCOUNT);
  Serial.print("SAMPLESPSEC: ");
  Serial.println(SAMPLESPSEC);
  Serial.print("CALCULATESAMPLES: ");
  Serial.println(CALCULATESAMPLES);
}
