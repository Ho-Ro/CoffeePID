/*
      _____ _ _       _       _____ _____ _____
     / ____(_) |     (_)     |  __ \_   _|  __ \
    | (___  _| |_   ___  __ _| |__) || | | |  | |
     \___ \| | \ \ / / |/ _` |  ___/ | | | |  | |
     ____) | | |\ V /| | (_| | |    _| |_| |__| |
    |_____/|_|_| \_/ |_|\__,_|_|   |_____|_____/

    A user-friendly temperature regulator for my
    Rancilio Silvia coffee machine.
    This setup is based on:
    - an ESP8266 microcontroller board
    - a NTC 10k thermo sensor at analog input A0
    - an SSR relay (pin variable "heaterPin") on output D1
    - an 230 V AC / 5 V DC power module for the ESP

    Author   : Ho-Ro
    Based on : @1024kilobyte
    Version  : 2.0
*/

// wifi stuff
#include <ESP8266WiFi.h>
#include "wifinetwork.h"

// webserver component
#include <ESP8266WebServer.h>

// mdns component
#include <ESP8266mDNS.h>

// over the air update
#include <ArduinoOTA.h>

// JSON library
#include <ArduinoJson.h>

// LittleFS
#include <LittleFS.h>

// declare webserver instance
ESP8266WebServer server( 80 );

// PID control
#include "PIDcontrol.h"

// slow PWM output
#include "PWMout.h"

// read real boiler sensor /  simulate boiler
#include "temperature.h"

// hardware pinout
const uint8_t heaterPin = D1;    // GPIO 05
const uint8_t heaterLedPin = D0; // GPIO 16
const uint8_t wifiLedPin = D4;   // GPIO 02
const bool ledOn = LOW;
const bool ledOff = HIGH;

// internal state variables
bool isHeating = false;
bool isStandby = false;

// WiFi configuration values
String preferredWiFiMode;
String wifiClientSSID, wifiClientPassword;
String wifiApSSID, wifiApPassword;

const char * SilviaPID = "SilviaPID";
const char * apPassword = "Rancilio";

// data for the PID
double pidKP = 25.0;
double pidKI = 1.0;
double pidKD = 100.0;
double pidBalance = 10; // pid output needed to hold constant target temperature
double pidOutput = 0;

double targetTemp = 100;
double currentTemp = 20;

double pidIntegral = 0;

const unsigned pidTimeStep = 1000;
const unsigned heaterTimeStep = 5000;

// input/output variables passed by reference, so they are updated automatically
// set output (pwm) range 0..100 % for use with PWMrelay
PIDcontrol heaterPID( &currentTemp, &targetTemp, &pidOutput,
                      0.0, 100.0,
                      pidKP, pidKI, pidKD );

// low-frequency PWM signal
PWMout heaterPWM( heaterPin, HIGH, heaterTimeStep );


// *********
// * SETUP *
// *********

void setup() {
    Serial.begin( 115200 );
    Serial.println();

    // init relay amd led pin
    pinMode( heaterPin, OUTPUT );
    digitalWrite( heaterPin, LOW );
    pinMode( heaterLedPin, OUTPUT );
    digitalWrite( heaterLedPin, ledOff );
    pinMode( wifiLedPin, OUTPUT );
    digitalWrite( wifiLedPin, ledOff );

    // set hostname
    WiFi.hostname( SilviaPID );

    // read config from file system to global variables
    readConfig();

    Serial.println( "* Configuration" );
    Serial.println( "  Preferred Wifi Mode: " + preferredWiFiMode );
    Serial.println( "  WiFi AP | SSID: " + wifiApSSID + " | Length: " + wifiApSSID.length() );
    Serial.println( "  AP Password: " + wifiApPassword + " | Length: " + wifiApPassword.length() );
    Serial.println( "  WiFi Client | SSID: " + wifiClientSSID + " | Length: " + wifiClientSSID.length() );
    // Serial.println( "  Client Password: " + wifiClientPassword
    //                 + " | Length: " + wifiClientPassword.length() );
    Serial.println( "  Brewing Temp: " + String( targetTemp ) );
    Serial.println( "  PID kP: " + String( pidKP ) );
    Serial.println( "  PID kI: " + String( pidKI ) );
    Serial.println( "  PID kD: " + String( pidKD ) );
    Serial.println( "  Balance: " + String( pidBalance ) );

    // update PID parameter
    heaterPID.setGains( pidKP, pidKI, pidKD );

    // if temperature is more than 5 K below or 1 K above setpoint,
    // PID output will be set to max or min respectively
    heaterPID.setBangBang( 5, 1 );

    // set PID update interval (default = 1000 ms)
    heaterPID.setTimeStep( pidTimeStep );

    // now start wifi
    if ( preferredWiFiMode == "client" ) {
        if ( wifiClientSSID.length() > 0 && wifiClientPassword.length() > 0 ) {
            // Wifi client mode
            WiFi.mode( WIFI_STA );

            // check if preferred SSID is available
            bool ssidFound = false;
            int networksFound = WiFi.scanNetworks();
            Serial.printf( "%d network(s) found\r\n", networksFound );
            for ( int i = 0; i < networksFound; i++ ) {
                if ( WiFi.SSID( i ) == wifiClientSSID )
                    ssidFound = true;
                Serial.printf( "%c %d: %s, Ch:%d (%ddBm) %s\r\n",
                               WiFi.SSID( i ) == wifiClientSSID ? '*' : ' ', i + 1,
                               WiFi.SSID( i ).c_str(), WiFi.channel( i ), WiFi.RSSI( i ),
                               WiFi.encryptionType( i ) == ENC_TYPE_NONE ? "open" : "" );
            }
            if ( ssidFound ) { // try to connect
                WiFi.begin( wifiClientSSID, wifiClientPassword );
                Serial.print( "* Connecting to \"" + wifiClientSSID + "\" " );
                unsigned waitCounter = 0;
                while ( WiFi.status() != WL_CONNECTED ) {
                    // this yields for 20 s, more than enough for the wifi to work
                    digitalWrite( wifiLedPin, ledOn ); // LED pulse
                    delay( 20 );
                    digitalWrite( wifiLedPin, ledOff );
                    delay( 480 );
                    Serial.print( "." );
                    waitCounter += 500;

                    // ~20 seconds later - maybe the credentials are wrong -> try AP mode
                    digitalWrite( wifiLedPin, ledOff );
                    if ( waitCounter >= 20000 ) {
                        Serial.println( "X Could not connect using given credentials" );
                        Serial.println( "X Restarting in accesspoint mode..." );

                        // reset wifi client config
                        preferredWiFiMode = "ap";
                        // DO NOT delete these values
                        //wifiClientSSID = "";
                        //wifiClientPassword = "";
                        writeConfig();

                        // be sure everything is written, reboot and restart as AP
                        delay( 500 );
                        ESP.restart();
                    }
                }
                // connected to wifiClientSSID

                digitalWrite( wifiLedPin, ledOff );

                Serial.println();
                Serial.println( "  Connected (IP: " + WiFi.localIP().toString() + ")" );

                // Port defaults to 8266
                // ArduinoOTA.setPort(8266);

                // Hostname defaults to esp8266-[ChipID]
                ArduinoOTA.setHostname( "silviapid" );

                // No authentication by default
                // ArduinoOTA.setPassword( "Rancilio" );

                // Password can be set with it's md5 hash value as well
                // create the hash with "echo -n Rancilio | md5sum"
                // ArduinoOTA.setPasswordHash( "37157b0597add960f4aa3bb3b334302c" );

                ArduinoOTA.onStart( []() {
                    String type;
                    if ( ArduinoOTA.getCommand() == U_FLASH ) {
                        type = "program";
                    } else { // U_FS
                        type = "data";
                        LittleFS.end();
                    }
                    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
                    Serial.println( "OTA: Start updating " + type );
                } );

                ArduinoOTA.onEnd( []() {
                    Serial.println( "\nEnd" );
                } );

                ArduinoOTA.onProgress( []( unsigned int progress, unsigned int total ) {
                    Serial.printf( "OTA Progress: %u%%\r", ( progress / ( total / 100 ) ) );
                } );

                ArduinoOTA.onError( []( ota_error_t error ) {
                    Serial.printf( "OTA Error[%u]: ", error );
                    if ( error == OTA_AUTH_ERROR ) {
                        Serial.println( "Auth Failed" );
                    } else if ( error == OTA_BEGIN_ERROR ) {
                        Serial.println( "Begin Failed" );
                    } else if ( error == OTA_CONNECT_ERROR ) {
                        Serial.println( "Connect Failed" );
                    } else if ( error == OTA_RECEIVE_ERROR ) {
                        Serial.println( "Receive Failed" );
                    } else if ( error == OTA_END_ERROR ) {
                        Serial.println( "End Failed" );
                    }
                } );

                ArduinoOTA.begin();

                Serial.println( "* OTA started on \"silviapid:8266\"" );
            } else {
                // preferred SSID not found, fall back to access point mode
                Serial.printf( "* SSID \"%s\" not found\n", wifiClientSSID.c_str() );
                WiFi.mode( WIFI_AP );
                WiFi.softAP( wifiApSSID, wifiApPassword );
                Serial.println( "  Fall-back: Started accesspoint \"" + wifiApSSID
                                + "\" with password \"" + wifiApPassword + "\"" );
                IPAddress myIP = WiFi.softAPIP();
                Serial.print( "  IP address: " );
                Serial.println( myIP );
            }
        } else {
            // reset wifi mode
            Serial.println( "X Invalid credentials" );
            Serial.println( "X Restarting in accesspoint mode..." );

            //wifiClientSSID = "";
            //wifiClientPassword = "";
            preferredWiFiMode = "ap";
            writeConfig();

            // be sure everything is written and now reboot
            delay( 500 );
            ESP.restart();
        }
    } else {
        // set default values
        if ( preferredWiFiMode != "ap" ) {
            preferredWiFiMode = "ap";
            writeConfig();
        }

        if ( wifiApSSID.length() == 0 ) {
            wifiApSSID = SilviaPID;
            wifiApPassword = apPassword;
            writeConfig();
        }

        // access point mode
        WiFi.mode( WIFI_AP );
        WiFi.softAP( wifiApSSID, wifiApPassword );
        Serial.println( "* Default: Started accesspoint \"" + wifiApSSID
                        + "\" with password \"" + wifiApPassword + "\"" );
        IPAddress myIP = WiFi.softAPIP();
        Serial.print( "  IP address: " );
        Serial.println( myIP );
    }

    // setup mDNS to be reachable by name
    if ( !MDNS.begin( "silviapid" ) ) {
        Serial.println( "X Error setting up mDNS responder!" );
    } else {
        Serial.println( "* mDNS responder started" );
    }

    // configure and start webserver
    // register root
    server.on( "/", handleRoot );

    // register ajax endpoints
    server.on( "/ajax_get_temp", getTemp );
    server.on( "/ajax_get_settings", getConfig );
    server.on( "/ajax_set_settings", setConfig );

    server.on( "/ajax_get_wifis", getWifis );

    server.on( "/get_status", getStatus );

    // all other routes and ressources
    server.onNotFound( handleWebRequests );

    server.begin();

    Serial.println( "* webserver started" );

    // add service to mDNS service discovery
    MDNS.addService( "http", "tcp", 80 );

    // finished

    // boot time - these should be the last lines in setup
    Serial.println( "* boot completed after " + String( millis() / 1000.0 ) + " seconds" );
    Serial.println( "********************************" );
    Serial.println( "* SilviaPID is up and running! *" );
    Serial.println( "********************************" );
}



// *************
// * MAIN LOOP *
// *************

void loop() {

    // ***************
    // * TEMP SENSOR *
    // ***************

    currentTemp = simulateTemperature( isHeating ); // runs at defined rate
    // currentTemp = getTemperature(); // runs at defined rate


    // ***************
    // * PID CONTROL *
    // ***************

    static bool nearSetpoint = false;
    static unsigned sensorError = 0;
    
    if ( currentTemp > 0 ) { // temperature input is fine

        if ( heaterPID.run() ) { // runs at defined rate, true if new output available

            // do some magic to improve the stability of the control and speed up setting
            // pidBalance is the needed PID output at stable setpoint temperature
            // only coming from integral part
            // prevent integral wind-up, limit the integral part to 150% of this value
            pidIntegral = constrain( heaterPID.getIntegral(), 0, 1.5 * pidBalance / pidKI );

            // if approaching the setpoint preset the integral with this value
            if ( heaterPID.atSetPoint( 0.5 ) ) { // only 0.5 degree away
                if ( ! nearSetpoint ) { // do this once when approaching the setpoint
                    nearSetpoint = true;
                    pidIntegral = 0.9 * pidBalance / pidKI; // limit integral to reduce overshoot
                }
            } else {
                nearSetpoint = false; // were still too far away
            }
            heaterPID.setIntegral( pidIntegral ); // feed back into PID

            heaterPWM.setPWM( pidOutput ); // forward the duty cycle to the relay pwm
            if ( sensorError )
                sensorError -=1; // we need 10 good readings after sensor error 
        }
    } else { // sensor error!
        sensorError = 10; // 10 s without error reset the status
    }

    tuneParameter();  // plot and edit PID parameter in Arduino serial plotter
    
    if (sensorError) { 
        heaterPID.stop();
        heaterPWM.setPWM( 0 );
        isHeating = false; // also for GUI
        digitalWrite( heaterPin, isHeating );     // switch off relay
        digitalWrite( heaterLedPin, !isHeating ); // switch off LED
    }

    isStandby = sensorError; // show on GUI
    
    
    // ******************
    // * HEATER CONTROL *
    // ******************

    heaterPWM.tick();  // runs at defined rate
    isHeating = heaterPWM.isActive();         // isHeating is also needed for the web UI
    // digitalWrite( heaterPin, isHeating );  // automatically done by heaterPWM.tick()
    digitalWrite( heaterLedPin, !isHeating ); // LED status update


    // *************
    // * FAST LOOP * - executed with every loop cycle
    // *************

    // help the esp to do its other tasks (wifi connections, aso.)
    yield();

    // over the air update
    ArduinoOTA.handle();

    // update mdns
    MDNS.update();

    // handle webserver requests
    server.handleClient();
}



// *************************************
// * TEMPERATURE PLOT / PARAMETER EDIT *
// *************************************

// reports target and current temperature for serial data plotter
// editParam shows PID parameter in top line of plotter
// checks for serial input, changes PID parameter accordingly
// writes config.json on request

void tuneParameter() {
    static bool editParam = true;
    static bool paramChanged = false;

    // process user input
    if ( Serial.available() > 0 ) {
        // read the incoming byte:
        byte incomingByte = Serial.read();
        if ( 'E' == incomingByte ) // enable parameter edit
            editParam = true;
        else if ( editParam ) {
            switch ( incomingByte ) {
                case 'e': // disable parameter edit
                    editParam = false;
                    break;
                case 'p': // decrease kP
                    pidKP -= 1;
                    if ( pidKP < 0 )
                        pidKP = 0;
                    paramChanged = true;
                    break;
                case 'P': // increase kP
                    pidKP += 1;
                    paramChanged = true;
                    break;
                case 'd':
                    pidKD -= 1;
                    if ( pidKD < 0 )
                        pidKD = 0;
                    paramChanged = true;
                    break;
                case 'D':
                    pidKD += 1;
                    break;
                case 'i':
                    pidKI -= 0.1;
                    if ( pidKI < 0 )
                        pidKI = 0;
                    paramChanged = true;
                    break;
                case 'I':
                    pidKI += 0.1;
                    paramChanged = true;
                    break;
                case 'b':
                    pidBalance -= 0.1;
                    if ( pidBalance < 0 )
                        pidBalance = 0;
                    paramChanged = true;
                    break;
                case 'B':
                    pidBalance += 0.1;
                    paramChanged = true;
                    break;
                case '@': // write current values to "config.json"
                    writeConfig();
                    paramChanged = false;
                    break;
            }
            heaterPID.setGains( pidKP, pidKI, pidKD );
        }
    }

    // plot temperature and show parameter
    static unsigned long lastMillis = millis();
    unsigned long now = millis();
    if ( now - lastMillis >= 1000 ) { // time reached
        lastMillis = now;
        if ( editParam ) {
            if ( paramChanged )
                Serial.print( "@" );
            Serial.print( "P" ); // display PID parameter and values in 1st trace ( targetTemp ) name
            Serial.print( int( pidKP ) );
            Serial.print( ";I" );
            Serial.print( pidKI );
            Serial.print( ";D" );
            Serial.print( int( pidKD ) );
            Serial.print( ";B" );
            Serial.print( pidBalance );
            Serial.print( ";i" );
            Serial.print( pidIntegral );
            Serial.print( ";o" );
            Serial.print( pidOutput  );
        } else {
            Serial.print( "Target Temperature=" );
            Serial.print( int( targetTemp ) );
        }
        Serial.print( ":" );
        Serial.print( targetTemp ); // plot target temp line
        Serial.print( ",Boiler Temperature=" );
        Serial.print( currentTemp );
        Serial.print( ":" );
        Serial.println( currentTemp ); // plot current temp line
    }
}



// ******************
// * HTTP ENDPOINTS *
// ******************

void handleRoot() {
    server.sendHeader( "Location", "/app", true );
    server.send( 302, "text/plain", "" );  // Found (Moved Temporarily)
}


void handleWebRequests() {
    // check for route
    String ressourceRequest = server.uri();

    LittleFS.begin(); // Filesystem mounten

    String dataFileName;
    String dataType = "text/plain";

    // to have custom routes, we need our own little router
    if ( ressourceRequest == "/app" ) {
        dataFileName = "/www/app.html";
    } else {
        dataFileName = "/www" + ressourceRequest;
    }

    // get file type
    if ( dataFileName.endsWith( ".svg" ) )
        dataType = "image/svg+xml";
    else if ( dataFileName.endsWith( ".js" ) )
        dataType = "application/javascript";
    else if ( dataFileName.endsWith( ".css" ) )
        dataType = "text/css";
    else if ( dataFileName.endsWith( ".html" ) )
        dataType = "text/html";
    else if ( dataFileName.endsWith( ".jpg" ) || dataFileName.endsWith( ".jpeg" ) )
        dataType = "image/jpeg";
    else if ( dataFileName.endsWith( ".ico" ) )
        dataType = "image/x-icon";
    else if ( dataFileName.endsWith( ".png" ) )
        dataType = "image/png";

    // load compressed version off the file if exists
    // the stream file method will set the content encoding header
    // this should be safe when we ensure that all files are gzipped
    dataFileName = dataFileName + ".gz";

    // set cache header for all files
    server.sendHeader( "Cache-Control", " max-age=31536000" );

    File dataFile = LittleFS.open( dataFileName.c_str(), "r" ); // Datei zum lesen öffnen

    digitalWrite( wifiLedPin, LOW ); // WiFi LED on
    if ( !dataFile ) {
        server.send( 404, "text/html", "<html><h1>File not found!</h1></html>" ); // Not Found
        digitalWrite( wifiLedPin, ledOff );
        return;
    }

    if ( server.streamFile( dataFile, dataType ) != dataFile.size() ) {
        // nothing we can really do about this...
    }
    dataFile.close();
    digitalWrite( wifiLedPin, ledOff );
}



// ******************
// * AJAX ENDPOINTS *
// ******************

void getTemp() { // http://silviapid/ajax_get_temp
    digitalWrite( wifiLedPin, LOW ); // WiFi LED on
    server.send( 200, "text/plain",  // OK
                 String( targetTemp ) + "|"
                 + String( currentTemp ) + "|"
                 + String( isHeating ) + "|"
                 + String( millis() ) + "|"
                 + String( isStandby ) );
    digitalWrite( wifiLedPin, ledOff );
}


// report the temperature and PID status, url: http://silviapid/get_status
void getStatus() {
    digitalWrite( wifiLedPin, LOW ); // WiFi LED on
    server.send( 200, "text/plain",  // OK
                 String( targetTemp ) + " "
                 + String( currentTemp ) + " "
                 + String( isHeating ) + " "
                 + String( millis() ) + " "
                 + String( isStandby ) + " "
                 + String( heaterPID.getError() * pidKP ) + " "
                 + String( heaterPID.getIntegral() * pidKI ) + " "
                 + String( heaterPID.getDError() * pidKD ) + " "
                 + String( pidBalance ) + " "
                 + String( pidOutput ) );
    digitalWrite( wifiLedPin, ledOff );
}


void getConfig() { // http://silviapid/ajax_get_config
    digitalWrite( wifiLedPin, LOW ); // WiFi LED on
    server.send( 200, "text/json",   // OK
                 "{ \"preferred_wifi_mode\": \"" + String( preferredWiFiMode )
                 + "\", \"wifi_ap_ssid\": \"" + String( wifiApSSID )
                 + "\", \"wifi_ap_password\": \"" + String( wifiApPassword )
                 + "\", \"wifi_client_ssid\": \"" + String( wifiClientSSID )
                 + "\", \"target_temp\": \"" + String( targetTemp )
                 + "\" }" );
    digitalWrite( wifiLedPin, ledOff );
}


void setConfig() {
    digitalWrite( wifiLedPin, LOW ); // WiFi LED on
    // check for reboot
    if ( server.arg( "reboot" ).length() > 0 ) {
        bool reboot = server.arg( "reboot" ) == "true";

        if ( reboot ) {
            Serial.println( "reboot" );
            server.send( 202 );  // Accepted

            // wait for the response to be received, else the client
            // doesn't receive our response and get a disconnect error
            delay( 800 );

            ESP.restart();
        }
    }

    // check for changed wifi credentials
    String preferred_wifi_mode = server.arg( "preferred_wifi_mode" );
    if ( preferred_wifi_mode.length() > 0 && preferred_wifi_mode != "null" ) {
        preferredWiFiMode = preferred_wifi_mode;
    }

    String wifi_ap_ssid = server.arg( "wifi_ap_ssid" );
    String wifi_ap_password = server.arg( "wifi_ap_password" );
    if ( wifi_ap_ssid.length() > 0 ) {
        wifiApSSID = wifi_ap_ssid;
        wifiApPassword = wifi_ap_password;
    }

    String wifi_client_ssid = server.arg( "wifi_client_ssid" );
    String wifi_client_password = server.arg( "wifi_client_password" );
    if ( wifi_client_ssid.length() > 0 ) {
        wifiClientSSID = wifi_client_ssid;
    }
    if ( wifi_client_password.length() > 0 ) {
        wifiClientPassword = wifi_client_password;
    }

    String target_temp = server.arg( "target_temp" );
    if ( target_temp.length() > 0 ) {
        targetTemp = target_temp.toFloat();
    }

    writeConfig();

    server.send( 202 );  // Accepted
    digitalWrite( wifiLedPin, ledOff );
}


void getWifis() { // http://silviapid/ajax_get_wifis
    digitalWrite( wifiLedPin, LOW ); // WiFi LED on
    byte numSsid = WiFi.scanNetworks();
    WiFiNetwork networks[numSsid];

    // print the network number and name for each network found:
    for ( int wifiCounter = 0; wifiCounter < numSsid; wifiCounter++ ) {
        networks[wifiCounter] = WiFiNetwork( WiFi.SSID( wifiCounter ), WiFi.RSSI( wifiCounter ),
                                             WiFi.channel( wifiCounter ),
                                             WiFi.encryptionType( wifiCounter ) );

        Serial.println( WiFi.SSID( wifiCounter ) + " " + WiFi.encryptionType( wifiCounter ) );
    }

    // "quick" sorting - as its around 1 ms in bad conditions, this is just fine
    for ( int sortRepeat = 0; sortRepeat < numSsid; sortRepeat++ ) {
        for ( int sortCounter = 0; sortCounter < numSsid - 1; sortCounter++ ) {
            if ( networks[sortCounter].RSSI < networks[sortCounter + 1].RSSI ) {
                WiFiNetwork tmpNetwork = networks[sortCounter];
                networks[sortCounter] = networks[sortCounter + 1];
                networks[sortCounter + 1] = tmpNetwork;
            }
        }
    }

    String jsonPayload = "{ \"wifis\":[";

    for ( int index = 0; index < numSsid; index++ ) {
        jsonPayload += "{ \"ssid\": \"" + networks[index].SSID
                       + "\", \"channel\": " + String( networks[index].Channel )
                       + ", \"rssi\": " + String( networks[index].RSSI )
                       + ", \"reception\": \"" + networks[index].Reception
                       + "\", \"encryption\": \"" + networks[index].Encryption
                       + "\"}";

        if ( index != numSsid - 1 ) {
            jsonPayload += ", ";
        }
    }

    jsonPayload += "]}";

    server.send( 200, "text/json", jsonPayload );  // OK
    digitalWrite( wifiLedPin, ledOff );
}



// ************************
// * CONFIG FILE HANDLING *
// ************************

void readConfig() {
    LittleFS.begin();

    if ( LittleFS.exists( "/config.json" ) ) {
        File config_file = LittleFS.open( "/config.json", "r" );

        DynamicJsonDocument doc( 512 );
        deserializeJson( doc, config_file.readString() );

        config_file.close();

        // ignore non existing keys in the config file
        if ( doc.containsKey( "preferred_wifi_mode" ) )
            preferredWiFiMode = ( const char * ) doc["preferred_wifi_mode"];
        if ( doc.containsKey( "wifi_client_ssid" ) )
            wifiClientSSID = ( const char * ) doc["wifi_client_ssid"];
        if ( doc.containsKey( "wifi_client_password" ) )
            wifiClientPassword = ( const char * ) doc["wifi_client_password"];
        if ( doc.containsKey( "wifi_ap_ssid" ) )
            wifiApSSID = ( const char * ) doc["wifi_ap_ssid"];
        if ( doc.containsKey( "wifi_ap_password" ) )
            wifiApPassword = ( const char * ) doc["wifi_ap_password"];
        if ( doc.containsKey( "target_temp" ) )
            targetTemp = ( float ) doc["target_temp"];
        if ( doc.containsKey( "pid_kp" ) )
            pidKP = ( float ) doc["pid_kp"];
        if ( doc.containsKey( "pid_ki" ) )
            pidKI = ( float ) doc["pid_ki"];
        if ( doc.containsKey( "pid_kd" ) )
            pidKD = ( float ) doc["pid_kd"];
        if ( doc.containsKey( "pid_balance" ) )
            pidBalance = doc["pid_balance"];
    }
}


void writeConfig() {
    DynamicJsonDocument doc( 512 );
    doc["preferred_wifi_mode"] = preferredWiFiMode;
    doc["wifi_client_ssid"] = wifiClientSSID;
    doc["wifi_client_password"] = wifiClientPassword;
    doc["wifi_ap_ssid"] = wifiApSSID;
    doc["wifi_ap_password"] = wifiApPassword;
    doc["target_temp"] = targetTemp;
    doc["pid_kp"] = pidKP;
    doc["pid_ki"] = pidKI;
    doc["pid_kd"] = pidKD;
    doc["pid_balance"] = pidBalance;

    LittleFS.begin();

    File config_file = LittleFS.open( "/config.json", "w+" ); // Datei zum schreiben öffnen;
    serializeJson( doc, config_file );
    config_file.close();
}
