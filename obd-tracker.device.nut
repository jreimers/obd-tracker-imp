#require "LSM9DS0.class.nut:1.1.0"

// Copyright (c) 2015 Electric Imp
// This file is licensed under the MIT License
// http://opensource.org/licenses/MIT

// Class for interfacing with an STN1110 over UART
// Enqueues commands and fires callbacks when replies are received or command times out
class STN1110 {
    
    _UART_TIMEOUT = 1; // UART sync read timeout in seconds
    _UART_MAX_BUFFER_LENGTH = 4096; // 4kB max buffer size before an error is thrown
    _NEWLINE = "\r";
    _NEWLINE_CHAR = 0x0D; // '\r' char
    _PROMPT_CHAR = 0x3E; // '>' char
    _END_OF_PACKET = "\r\r>";
    _COMMAND_RESET = "ATZ";
    _COMMAND_DISABLE_ECHO = "ATE0";
    _COMMAND_SET_BAUD_RATE = "STBR";
    _COMMAND_SUCCESS_REPLY = "OK";
    _END_OF_TRANSMISSION = 0x04;
    _PID_REPLY_MODE_OFFSET = 0x40; // PID responses have the mode offset by 0x40 from the request

    _uart = null; // UART the STN1110 is connected to
    _uartCurrentBaud = null;
    _uartReadTimeoutStart = null; // hardware.millis() that sync read timeouts are measured from
    _buffer = null; // UART recieve buffer
    _commandQueue = null; // queue of Command objects awaiting execution
    _activeCommand = null; // the currently executing command
    _initialized = null; // constructor completion success flag
    _elmVersion = null; // the version of the ELM emulator returned by the STN1110 on reset
    _errorCallback = null; // callback function for errors occuring after initialization
    
    // Constructs a new instance of STN1110
    // This WILL reset your UART configuration for the supplied uart parameter
    // When this method returns the UART interface is ready to use, unless an error is thrown
    // Optional baud parameter can be used for initial connection if the STN1110 has a baud rate
    // other than the default 9600 stored in it's EEPROM
    constructor(uart, baud=9600) {
        _initialized = false;
        _buffer = "";
        _commandQueue = [];
        _uart = uart;
        _uartCurrentBaud = baud;
        _uart.configure(baud, 8, PARITY_NONE, 1, NO_CTSRTS, _uartCallback.bindenv(this)); // configure uart, register callback
        reset(); // reset STN1110
        _initialized = true;
    }

    // Resets the STN1110 and blocks until the STN1110 is ready to accept commands or times out
    function reset() {
        // write reset command
        _uart.write(_COMMAND_RESET + _NEWLINE);
        try {
            // handle inconsistency in output between hard and soft reset
            local echoOrVersion = _readLineSync();
            if(echoOrVersion == _COMMAND_RESET) {
                _elmVersion = _readLineSync(); // it was an echo, read the next line for version
            } else {
                _elmVersion = echoOrVersion; // no echo, it's the version
            }
            _uart.write(_COMMAND_DISABLE_ECHO + _NEWLINE); // disable echo
            _readLineSync(); // read echo of disable echo command
            local response;
            if((response = _readLineSync()) != _COMMAND_SUCCESS_REPLY) { // disable echo failed
                throw "Unexpected response when disabling echo '" + response + "'";
            }
            _eatPromptSync(); // eat the prompt
        } catch(error) { // likely something timed out
            throw "Error initializing STN1110 UART interface: " + error;
        }
    }

    // returns the version string of the ELM emulator provided by the STN1110 on reset
    function getElmVersion() {
        return _elmVersion;
    }
    
    function getBaudRate() {
        return _uartCurrentBaud;
    }
    
    function setBaudRate(baud) {
        if(_activeCommand != null || _commandQueue.len() > 0) {
            throw "Cannot change baud rate while commands are executing"
        }
        _uart.write(_COMMAND_SET_BAUD_RATE + " " + baud + _NEWLINE); // write set baud rate command
        if(_readLineSync() != _COMMAND_SUCCESS_REPLY) { // STN1110 will reply OK if the baud rate is valid
            throw "Error setting baud rate to '" + baud + "'";
        }
        // reconfigure our uart
        _uart.configure(baud, 8, PARITY_NONE, 1, NO_CTSRTS, _uartCallback.bindenv(this)); // reconfigure uart, register callback
        _uart.write(_NEWLINE) // write newline so the STN1110 knows the baud rate change was successful
        _readLineSync() // read STN1110 version string
        if(_readLineSync() != _COMMAND_SUCCESS_REPLY) { // make sure we can receive OK from the STN1110
            // try to restore to previous baud then throw error
            _uart.configure(_uartCurrentBaud, 8, PARITY_NONE, 1, NO_CTSRTS, _uartCallback.bindenv(this));
            throw "Error setting baud rate to '" + baud + "'";
        }
        // everything went well, keep track of the new baud
        _uartCurrentBaud = baud;
    }

    // Executes the command string 'command' with timeout 'timeout' seconds and calls 'callback' with the result
    function execute(command, timeout, callback) {
        local cmd = { 
            "command": command, 
            "timeout": timeout,
            "callback": callback
        };
        _commandQueue.append(cmd);
        if(_activeCommand == null) {
            _executeNextInQueue();
        }
    }

    // Pass a callback function to be called if an error occurs after initialization and no PID callbacks are registered to receive the error
    function onError(callback) {
        _errorCallback = callback;
    }
    
    /* Private methods */

    // executes the next command in the queue
    function _executeNextInQueue() {
        if(_commandQueue.len() > 0) {
            _activeCommand = _commandQueue.remove(0);
            _activeCommand.timer <- imp.wakeup(_activeCommand.timeout, _handleTimeout.bindenv(this));
            _uart.write(_activeCommand.command + _NEWLINE);
        }
    }

    // clears the active command and cancels its timeout timer
    function _clearActiveCommand() {
        if(_activeCommand == null) {
            return;
        }
        imp.cancelwakeup(_activeCommand.timer);
        _activeCommand = null;
    }

    // blocks until it reads a > or times out
    function _eatPromptSync() {
        if(_uartReadTimeoutStart == null) {
            _uartReadTimeoutStart = hardware.millis();
        }
        while (_uart.read() != _PROMPT_CHAR) {
            if((hardware.millis() - _uartReadTimeoutStart) > _UART_TIMEOUT*1000) {
                throw "UART read timed out while waiting for prompt";
            }
        }
    }
    
    // blocks until it reads a CR with a non empty preceeding line then returns the line, or times out
    function _readLineSync() {
        _buffer = "";
        local char = null;
        if(_uartReadTimeoutStart == null) {
            _uartReadTimeoutStart = hardware.millis();
        }
        while ((char = _uart.read()) != _NEWLINE_CHAR) { // read until newline
            if(char != -1) { // data was read
                _buffer += format("%c", char);
                _buffer = strip(_buffer);
            } else {
                if((hardware.millis() - _uartReadTimeoutStart) > _UART_TIMEOUT*1000) {
                    throw "UART read timed out while waiting for newline";
                }
            }
        }
        if(_buffer.len() > 0) { // read a line with something on it
            local localBuf = _buffer;
            _buffer = "";
            _uartReadTimeoutStart = null;
            return localBuf;
        }
        return _readLineSync(); // line was empty, try again
    }

    // uart callback method, called when data is available
    function _uartCallback() {
        if(!_initialized) {
            return;
        }
        _buffer += _uart.readstring();
        local packets = _packetize_buffer();
        for (local i = 0; i < packets.len(); i++) {
            _parse(packets[i]);
        }
    }

    // parse the uart buffer into an array of packets and remove those packets from the buffer
    function _packetize_buffer() {
        local index;
        local packets = [];
        while((index = _buffer.find(_END_OF_PACKET)) != null) { // find each end of packet sequence
            local packet = _buffer.slice(0, index);
            packets.push(packet);
            _buffer = _buffer.slice(packet.len() + 3, _buffer.len());
        }
        // after packetizing buffer check if it is too long
        if(_buffer.len() > _UART_MAX_BUFFER_LENGTH) {
            _error("UART buffer exceeded max length");
            _buffer = "" // clear buffer
        }
        return packets;
    }

    // call the callback for a received packet then move on or report an error if one is not registered
    function _parse(packet) {
        //server.log(packet)
        if(_activeCommand != null && "callback" in _activeCommand && _activeCommand.callback != null) { // make sure there is something to call
            _activeCommand.callback({"msg" : packet}); // execute the callback
            // execute next command
            _clearActiveCommand();
            _executeNextInQueue();
        } else {
            _error("No callback registered for response '" + _buffer + "'"); // got a response with no active command
        }
    }

    // command timeout handler 
    function _handleTimeout() {
        if(_activeCommand != null && "callback" in _activeCommand && _activeCommand.callback != null) {
            _activeCommand.callback({"err": "command '" + _activeCommand.command + "' timed out"}); // callback with error
        }
        _uart.write(_END_OF_TRANSMISSION); // send EOT char to kill command
        // execute next command
        _clearActiveCommand();
        _executeNextInQueue();
    }

    // call error callback if one is registered
    function _error(errorMessage) {
        if(_errorCallback != null) {
            _errorCallback(errorMessage);
        }
    }
}

// Provides a high level interface for accessing vehicle data over OBD-II
class VehicleInterface extends STN1110 {
   
    ENGINE_RPM = 0x010C;
    VEHICLE_SPEED = 0x010D;
    THROTTLE_POSITION = 0x0111;
    COOLANT_TEMPERATURE = 0x0105;
    FUEL_PRESSURE = 0x010A;
    INTAKE_AIR_TEMPERATURE = 0x010F;
    ENGINE_RUNTIME = 0x011F;
        
    _transforms = {};
    _callbacks = null;

    function constructor(uart) {
        _callbacks = {};
        _transforms[ENGINE_RPM] <- function(data) {
            return ((data[0]*256)+data[1])/4;
        };
        _transforms[VEHICLE_SPEED] <- function(data) {
            return data[0];
        };
        _transforms[THROTTLE_POSITION] <- function(data) {
            return data[0]*100/255;
        };
        _transforms[COOLANT_TEMPERATURE] <- function(data) {
            return data[0]-40;
        };
        _transforms[FUEL_PRESSURE] <- function(data) {
            return data[0]*3;
        };
        _transforms[INTAKE_AIR_TEMPERATURE] <- function(data) {
            return data[0]-40;
        };
        _transforms[ENGINE_RUNTIME] <- function(data) {
            return (data[0]*256)+data[1];
        };
        base.constructor(uart);
    }

    // reads a PID once and executes callback with the resulting data
    function read(pid, callback) {
        _read(pid, (function(result) {
            _callback(_applyTransform(pid, result));
        }).bindenv(this));
    }

    // reads a PID every 'period' seconds calling 'callback' with the resulting data
    function subscribe(pid, callback, period) {
        _callbacks[pid] <- callback;
        _schedule(pid, period);
    }

    // unsubscribes the callback, if any, for PID 'pid'
    function unsubcribe(pid) {
        if(pid in _callbacks) {
            delete _callbacks[pid];
        }
    }

    // requests a PID and interprets the result
    function _read(pid, callback) {
        if(pid > 0xFFFF) { // sanity check
            callback({"err": "Invalid PID '" + pid + "'"});
        }
        execute(format("%04X", pid), 1, (function(result) { // format command back to hex string and execute
            if("err" in result) {
                callback(result);
                return;
            }
            local str_bytes = split(result["msg"], " ");
            local bytes = [];
            // convert response string back into bytes
            for(local i = 0; i < str_bytes.len(); i++) {
                bytes.append(this._hexToInt(str_bytes[i]));
            }
            // check that the first two bytes match what we sent (mode + 0x40 and PID code)
            if(((bytes[0] - _PID_REPLY_MODE_OFFSET) != ((pid & 0xFF00) >> 8)) || (bytes[1] != (pid & 0x00FF))) {
              callback({"err": "Response does not match requested PID"});
              return;
            }
            callback({"msg": bytes.slice(2, bytes.len())}); // skip the first two bytes which don't contain any response data
        }).bindenv(this));
    }

    // schedules a PID to be checked every 'period' seconds as long as a callback is registered in the _callbacks table
    function _schedule(pid, period) {
        if(pid in _callbacks) { // check
            _read(pid, (function(result) {
                if(pid in _callbacks) { // check again because async
                    _callbacks[pid](_applyTransform(pid, result)); // apply transform on the response data and execute callback
                }
            }).bindenv(this));
            imp.wakeup(period, (function() { _schedule(pid, period) }).bindenv(this)); // do it again
        }
    }

    // converts the response byte(s) to a value in the PIDs associates units, if hte response is valid and there is a transform for that PID
    function _applyTransform(pid, result) {
        if(!("err" in result) && pid in _transforms) {
            return { "msg": _transforms[pid](result["msg"]) };
        }
        return result;
    }

    // converts a hex string to an integer
    function _hexToInt(hex) {
        local result = 0;
        local shift = hex.len() * 4;
        for(local d=0; d<hex.len(); d++) {
            local digit;
            if(hex[d] >= 0x61) {
                digit = hex[d] - 0x57;
            } else if(hex[d] >= 0x41) {
                digit = hex[d] - 0x37;
            } else {
                digit = hex[d] - 0x30;
            }
            shift -= 4;
            result += digit << shift;
        }
        return result;
    }
}



// Driver class for the PA6H GPS module
// http://www.adafruit.com/datasheets/GlobalTop-FGPMMOPA6H-Datasheet-V0A.pdf
// Used in V3 of the Adafruit "Ultimate" GPS Breakout
// https://www.adafruit.com/products/746
class PA6H {
    
    // GGA: Time, position and fix type data.
    // GSA: GPS receiver operating mode, active satellites used in the
    //      position solution and DOP values.
    // GSV: The number of GPS satellites in view satellite ID numbers,
    //      elevation, azimuth, and SNR values.
    // RMC: Time, date, position, course and speed data. Recommended
    //      Minimum Navigation Information.
    // VTG: Course and speed information relative to the ground
    
    // different commands to set the update rate from once a second (1 Hz) to 10 times a second (10Hz)
    // Note that these only control the rate at which the position is echoed, to actually speed up the
    // position fix you must also send one of the position fix rate commands below too.
    static PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ      = "$PMTK220,10000*2F"; // Once every 10 seconds, 100 millihertz.
    static PMTK_SET_NMEA_UPDATE_1HZ                 = "$PMTK220,1000*1F";
    static PMTK_SET_NMEA_UPDATE_5HZ                 = "$PMTK220,200*2C";
    static PMTK_SET_NMEA_UPDATE_10HZ                = "$PMTK220,100*2F";
    // Position fix update rate commands.
    static PMTK_API_SET_FIX_CTL_100_MILLIHERTZ      = "$PMTK300,10000,0,0,0,0*2C"; // Once every 10 seconds, 100 millihertz.
    static PMTK_API_SET_FIX_CTL_1HZ                 = "$PMTK300,1000,0,0,0,0*1C";
    static PMTK_API_SET_FIX_CTL_5HZ                 = "$PMTK300,200,0,0,0,0*2F";
    // Can't fix position faster than 5 times a second!
    
    static PMTK_SET_BAUD_57600                      = "$PMTK251,57600*2C";
    static PMTK_SET_BAUD_9600                       = "$PMTK251,9600*17";
    
    // turn on only the second sentence (GPRMC)
    static PMTK_SET_NMEA_OUTPUT_RMCONLY             = "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29";
    // turn on GPRMC and GGA
    static PMTK_SET_NMEA_OUTPUT_RMCGGA              = "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28";
    // turn on ALL THE DATA
    static PMTK_SET_NMEA_OUTPUT_ALLDATA             = "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28";
    // turn off output
    static PMTK_SET_NMEA_OUTPUT_OFF                 = "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28";
    
    // to generate your own sentences, check out the MTK command datasheet and use a checksum calculator
    // such as the awesome http://www.hhhh.org/wiml/proj/nmeaxor.html
    static PMTK_LOCUS_STARTLOG                      = "$PMTK185,0*22";
    static PMTK_LOCUS_STOPLOG                       = "$PMTK185,1*23";
    static PMTK_LOCUS_STARTSTOPACK                  = "$PMTK001,185,3*3C";
    static PMTK_LOCUS_QUERY_STATUS                  = "$PMTK183*38";
    static PMTK_LOCUS_ERASE_FLASH                   = "$PMTK184,1*22";
    static LOCUS_OVERLAP                            = 0;
    static LOCUS_FULLSTOP                           = 1;
    
    static PMTK_ENABLE_SBAS                         = "$PMTK313,1*2E";
    static PMTK_ENABLE_WAAS                         = "$PMTK301,2*2E";
    
    // standby command & boot successful message
    static PMTK_STANDBY                             = "$PMTK161,0*28";
    static PMTK_STANDBY_SUCCESS                     = "$PMTK001,161,3*36";  // Not needed currently
    static PMTK_AWAKE                               = "$PMTK010,002*2D";
    
    // ask for the release and version
    static PMTK_Q_RELEASE                           = "$PMTK605*31";
    
    // request for updates on antenna status 
    static PGCMD_ANTENNA                            = "$PGCMD,33,1*6C"; 
    static PGCMD_NOANTENNA                          = "$PGCMD,33,0*6D"; 
    
    static DEFAULT_BAUD  = 9600;

    // pins and hardware
    _uart   = null;
    _fix    = null;
    _en     = null;
    
    _uart_baud = null;
    _uart_buffer = "";
    
    // vars
    _last_pos_data = {};
    
    _position_update_cb = null;
    _dop_update_cb      = null;
    _sats_update_cb     = null;
    _rmc_update_cb      = null;
    _vtg_update_cb      = null;
    _ant_status_update_cb = null;
    
    // -------------------------------------------------------------------------
    constructor(uart, en = null, fix = null) {
        _uart   = uart;
        _en     = en;
        _fix    = fix;
        
        _uart_baud = DEFAULT_BAUD;
        _uart.configure(_uart_baud, 8, PARITY_NONE, 2, NO_CTSRTS, _uartCallback.bindenv(this));
    }
    
    // -------------------------------------------------------------------------
    function _sendCmd(cmdStr) {
        _uart.write(cmdStr);
        _uart.write("\x0D\x0A");
        _uart.flush();
    }   
    
    // -------------------------------------------------------------------------
    // Parse a UTC timestamp from 
    // 064951.000 hhmmss.sss
    // Into
    // 06:09:51.000
    function _parseUTC(ts) {
        local result = "";
        result = result + (ts.slice(0,2)+ ":" + ts.slice(2,4) + ":" + ts.slice(4,ts.len()));
        return result;
    }
    
    // -------------------------------------------------------------------------
    // Parse a lat/lon coordinate from
    // 
    // ddmm.mmmm or dddmm.mmmm
    //
    // uhh.. this is a stub right now
    function _parseCoordinate(coordinatestring) {
        return coordinatestring;
    }
    
    // -------------------------------------------------------------------------
    function _uartCallback() {
        //server.log(_uart_buffer);
        _uart_buffer += _uart.readstring(80);
        local packets = split(_uart_buffer,"$");
        for (local i = 0; i < packets.len(); i++) {
            // make sure we can see the end of the packet before trying to parse
            if (packets[i].find("\r\n")) {
                try {
                    local len = packets[i].len()
                    _parse(packets[i]);
                    _uart_buffer = _uart_buffer.slice(len + 1,_uart_buffer.len());
                } catch (err) {
                   _uart_buffer = "";
                   server.log(err+", Pkt: "+packets[i]);
                }
            }
        }
    }

    // -------------------------------------------------------------------------
    function _parse(packetstr_raw) {
        //server.log(packetstr_raw);
        // "PMTKxxxx" is a system command packet; ignore for now
        if (packetstr_raw.find("PMTK") != null) { return }
        packetstr_raw = split(strip(packetstr_raw),"*");
        local packetstr = packetstr_raw[0];
        //server.log(packetstr);
        local checksum = packetstr_raw[1];
        //server.log(checksum);
        
        // string split swallows repeated split characters, 
        // so workaround with string find for now
        //local fields = split(packetstr,",");
        local fields = [];
        local start = 0;
        local end = 0;
        do {
            end = packetstr.find(",",start);
            if (end != null) {
                fields.push(packetstr.slice(start,end));
                start = end+1;
            }
        } while (end != null);
        fields.push(packetstr.slice(start,packetstr.len()));
        
        local hdr = fields[0];
        switch (hdr) {
            case "GPGGA":
                // time, position, and fix data
                // Ex: "$GPGGA,064951.000,2307.1256,N,12016.4438,E,1,8,0.95,39.9,M,17.8,M,,*65 "
                // UTC Time 064951.000 hhmmss.sss
                _last_pos_data.time <- _parseUTC(fields[1]);
                // Latitude 2307.1256 ddmm.mmmm
                _last_pos_data.lat <- _parseCoordinate(fields[2]);
                // N/S Indicator N N=north or S=south
                _last_pos_data.ns <- fields[3];
                // Longitude 12016.4438 dddmm.mmmm
                _last_pos_data.lon <- _parseCoordinate(fields[4]);
                // E/W Indicator E E=east or W=west
                _last_pos_data.ew <- fields[5];
                // Position Fix
                _last_pos_data.fix <- fields[6];
                // Satellites Used 8 Range 0 to 14
                _last_pos_data.sats_used <- fields[7].tointeger();
                // HDOP 0.95 Horizontal Dilution of Precision
                _last_pos_data.hdop <- fields[8].tofloat();
                // MSL Altitude 39.9 meters Antenna Altitude above/below mean-sea-level
                _last_pos_data.msl <- fields[9].tofloat();
                // Units M meters Units of antenna altitude
                _last_pos_data.units_alt <- fields[10].tofloat();
                // Geoidal Separation 17.8 meters
                _last_pos_data.geoidal_sep <- fields[11].tofloat();
                // Units M meters Units of geoids separation
                _last_pos_data.units_sep <- fields[12];
                // Age of Diff. Corr. second Null fields when DGPS is not used
                _last_pos_data.diff_corr <- fields[13];

                if (_position_update_cb) _position_update_cb(_last_pos_data);
                break;
            case "GPGSA":
                // DOP and Active Satellites Data
                // Ex: "$GPGSA,A,3,29,21,26,15,18,09,06,10,,,,,2.32,0.95,2.11*00 "
                // "M" = manual (forced into 2D or 3D mode)
                // "A" = 2C Automatic, allowed to auto-switch 2D/3D
                _last_pos_data.mode1 <- fields[1];
                // "1" = Fix not available
                // "2" = 2D (<4 SVs used)
                // "3" = 3D (>= 4 SVs used)
                _last_pos_data.mode2 <- fields[2];
                // Satellites Used on Channel 1
                _last_pos_data.sats_used_1 <- fields[3].tointeger();
                _last_pos_data.sats_used_2 <- fields[4].tointeger();
                _last_pos_data.sats_used_3 <- fields[5].tointeger();
                _last_pos_data.sats_used_4 <- fields[6].tointeger();                
                _last_pos_data.sats_used_5 <- fields[7].tointeger();
                _last_pos_data.sats_used_6 <- fields[8].tointeger();                
                _last_pos_data.sats_used_7 <- fields[9].tointeger();
                _last_pos_data.sats_used_8 <- fields[10].tointeger();
                _last_pos_data.sats_used_9 <- fields[11].tointeger();
                _last_pos_data.sats_used_10 <- fields[12].tointeger();                
                _last_pos_data.sats_used_11 <- fields[13].tointeger();
                _last_pos_data.sats_used_12 <- fields[14].tointeger();
                // Positional Dilution of Precision
                _last_pos_data.pdop <- fields[15].tofloat();
                // Horizontal Dilution of Precision
                _last_pos_data.hdop <- fields[16].tofloat();
                // Vertical Dilution of Precision
                _last_pos_data.vdop <- fields[17].tofloat();

                if (_dop_update_cb) _dop_update_cb(_last_pos_data);
                break;
            case "GPGSV":
                // GNSS Satellites in View
                // Ex: "$GPGSV,3,1,09,29,36,029,42,21,46,314,43,26,44,020,43,15,21,321,39*7D"
                // Number of Messages (3)
                local num_messages = fields[1].tointeger();
                local message_number = fields[2].tointeger();
                _last_pos_data.sats_in_view <- fields[3].tointeger();
                if (!"sats" in _last_pos_data) _last_pos_data.sats <- [];
                local i = 4; // current index in fields
                while (i < (fields.len() - 4)) {
                    local sat = {};
                    sat.id <- fields[i++].tointeger();
                    sat.elevation <- fields[i++].tofloat();
                    sat.azimuth <- fields[i++].tofloat();
                    sat.snr <- fields[i++].tofloat();
                    _last_pos_data.sats.push(sat);
                }

                if (_sats_update_cb) _sats_update_cb(_last_pos_data);
                break;
            case "GPRMC":
                // Minimum Recommended Navigation Information
                // Ex: "$GPRMC,064951.000,A,2307.1256,N,12016.4438,E,0.03,165.48,260406,3.05,W,A*2C "
                // UTC time hhmmss.sss
                _last_pos_data.time <- _parseUTC(fields[1]);
                // Status A=Valid V=Not Valid
                _last_pos_data.status <- fields[2];
                // ddmm.mmmm
                _last_pos_data.lat <- _parseCoordinate(fields[3]);
                // N/S Indicator
                _last_pos_data.ns <- fields[4];
                // ddmm.mmmm
                _last_pos_data.lon <- _parseCoordinate(fields[5]);
                // E/W Indicator
                _last_pos_data.ew <- fields[6];
                // Ground speed in knots
                _last_pos_data.gs_knots <- fields[7].tofloat();
                // Course over Ground, Degrees True
                _last_pos_data.true_course <- fields[8].tofloat();
                // Date, ddmmyy
                _last_pos_data.date <- fields[9];
                // Magnetic Variation (Not available)
                _last_pos_data.mag_var <- fields[10];
                // Mode (A = Autonomous, D = Differential, E = Estimated)
                _last_pos_data.mode <- fields[11];
                
                if (_rmc_update_cb) _rmc_update_cb(_last_pos_data);
                break;
            case "GPVTG":
                // Course and Speed information relative to ground
                // Ex: "$GPVTG,165.48,T,,M,0.03,N,0.06,K,A*37 "
                // Measured Heading, Degrees
                _last_pos_data.true_course <- fields[1].tofloat();
                // Course Reference (T = True, M = Magnetic)
                _last_pos_data.course_ref <- fields[2];
                // _last_pos_data.course_2 <- fields[3];
                // _last_pos_data.ref_2 <- fields[4];
                // Ground Speed in Knots
                _last_pos_data.gs_knots <- fields[5].tofloat();
                // Ground Speed Units, N = Knots
                //_last_pos_data.gs_units_1 <- fields[6];
                // Ground Speed in km/hr
                _last_pos_data.gs_kmph <- fields[7].tofloat();
                // Ground Speed Units, K = Km/Hr
                //_last_pos_data.gs_units_2 <- fields[8];
                // Mode (A = Autonomous, D = Differential, E = Estimated)
                _last_pos_data.mode <- fields[9];

                if (_vtg_update_cb) _vtg_update_cb(_last_pos_data);
                break;
            case "PGTOP":
                // Antenna Status Information
                // Ex: "$PGTOP,11,3 *6F"
                // Function Type (??)
                //_last_pos_data.function_type <- fields[1];
                // Antenna Status
                // 1 = Active Antenna Shorted
                // 2 = Using Internal Antenna
                // 3 = Using Active Antenna
                _last_pos_data.ant_status <- fields[2].tointeger();
                
                if (_ant_status_update_cb) _ant_status_update_cb(_last_pos_data);
                break;
            case "PGSA": 
                // ???
                break;
            case "PGACK":
                // command ACK
                // do nothing ...?
                server.log("ACK: "+packetstr);
                break;
            case "GPGLL":
                // no idea what this is
                break;
            default:
                throw "Unrecognized Header";
            }
    }   
    
    // -------------------------------------------------------------------------
    function enable() {
        if (_en) _en.write(1);
    }
    
    // -------------------------------------------------------------------------
    function wakeup() {
        _uart.write(" ");
    }
    
    // -------------------------------------------------------------------------
    function standby() {
       _sendCmd(PMTK_STANDBY);
    }
     
    // -------------------------------------------------------------------------
    function disable() {
        if (_en) _en.write(0);
    }
    
    // -------------------------------------------------------------------------
    function setBaud(baud) {
        if (baud == _uart_baud) return;
        if ((baud != 9600) && (baud != 57600)) throw format("Unsupported baud (%d); supported rates are 9600 and 57600",baud);
        if (baud == 57600) _sendCmd(PMTK_SET_BAUD_57600);
        else _sendCmd(PMTK_SET_BAUD_9600);
        _uart_baud = baud;
        _uart.configure(_uart_baud, 8, PARITY_NONE, 1, NO_CTSRTS, _uartCallback.bindenv(this));
    }
    
    // -------------------------------------------------------------------------
    function hasFix() {
        if (!_fix) throw "hasFix called but no Fix Pin provided to PA6H constructor" 
        return  _fix.read();
    }
    
    // -------------------------------------------------------------------------
    function setPositionCallback(cb) {
        _position_update_cb = cb;
    }
    
    // -------------------------------------------------------------------------
    function setDopCallback(cb) {
        _dop_update_cb = cb;
    }
    
    // -------------------------------------------------------------------------
    function setSatsCallback(cb) {
        _sats_update_cb = cb;
    }    

    // -------------------------------------------------------------------------
    function setRmcCallback(cb) {
        _rmc_update_cb = cb;
    }

    // -------------------------------------------------------------------------
    function setVtgCallback(cb) {
        _vtg_update_cb = cb;
    }
    
    // -------------------------------------------------------------------------
    function setAntStatusCallback(cb) {
        _ant_status_update_cb = cb;
    }
    
    // -------------------------------------------------------------------------
    // Controls how frequently the GPS tells us the latest position solution
    // This does not control the rate at which solutions are generated; use setUpdateRate to change that
    // Max Rate 10 Hz (0.1s per report)
    // Min Rate 100 mHz (10s per report)
    // Input: rateSeconds - the time between position reports from the GPS
    // Return: None
    function setReportingRate(rateSeconds) {
        if (rateSeconds <= 0.1) {
            _sendCmd(PMTK_SET_NMEA_UPDATE_10HZ);
        } else if (rateSeconds <= 0.5) {
            _sendCmd(PMTK_SET_NMEA_UPDATE_5HZ);
        } else if (rateSeconds <= 1) {
            _sendCmd(PMTK_SET_NMEA_UPDATE_1HZ);
        } else {
            _sendCmd(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ);
        }
    }
    
    // -------------------------------------------------------------------------
    // Controls how frequently the GPS calculates a position solution
    // Max rate 5Hz (0.2s per solution)
    // Min rate 100 mHz (10s per solution)
    // Input: rateSeconds - time between solutions by the GPS
    // Return: None
    function setUpdateRate(rateSeconds) {
        if (rateSeconds <= 0.2) {
            _sendCmd(PMTK_API_SET_FIX_CTL_5HZ);
        } else if (rateSeconds <= 1) {
            _sendCmd(PMTK_API_SET_FIX_CTL_1HZ);
        } else {
            _sendCmd(PMTK_API_SET_FIX_CTL_100_MILLIHERTZ);
        } 
    }
    
    // -------------------------------------------------------------------------
    // Set mode to RMC (rec minimum) and GGA (fix) data, incl altitude
    function setModeRMCGGA() {
        _sendCmd(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    }
    
    // -------------------------------------------------------------------------
    // Set mode to RMC (rec minimum) ONLY: best for high update rates
    function setModeRMC() {
        _sendCmd(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    }
    
    // -------------------------------------------------------------------------
    // Set mode to ALL. This will produce a lot of output...
    function setModeAll() {
        _sendCmd(PMTK_SET_NMEA_OUTPUT_ALLDATA);
    }
    
    // -------------------------------------------------------------------------
    function getPosition() {
        return _last_pos_data;    
    }
}


class BatchLogger {
    _log = null // log table
    _logUpdated = null // table of flags indicating if that log has been updated this cycle yet

    constructor() {
        this._log = {}
        this._logUpdated = {}        
    }

    // returns a function that accepts one parameter to be logged with name "key" when called
    function subscribe(key) {
        _logUpdated[key] <- false
        return (function(value) {
            if("msg" in value) { // if there is an error just reuse the last value
                this._log[key] <- value["msg"]
                this._logUpdated[key] = true
                this._checkAndLog()
            }
        }).bindenv(this)
    }
    // check if all log keys have been updated then send to server
    function _checkAndLog() {
        foreach(k,v in _logUpdated) {
            if(v == false) {
                return // bail if any values haven't been updated yet
            }
        }
        // post the log
        agent.send("log", _log)
        // clear all the updated flags
        foreach(k,v in _logUpdated) {
            _logUpdated[k] = false
        }
    }
}


logger <- BatchLogger()
speedLogger <- logger.subscribe("speed")
rpmLogger <- logger.subscribe("rpm")
throttleLogger <- logger.subscribe("throttle position")
airTempLogger <- logger.subscribe("air temperature")
coolantTempLogger <- logger.subscribe("coolant temperature")
fuelPressureLogger <- logger.subscribe("fuel pressure")

gpsLogger <- logger.subscribe("gps")
accelLogger <- logger.subscribe("accel")
timeLogger <- logger.subscribe("time")


car <- VehicleInterface(hardware.uartQRPW);
car.onError(function(err) {
    server.log("error: " + err)
})

car.subscribe(car.VEHICLE_SPEED, speedLogger, 5);
car.subscribe(car.ENGINE_RPM, rpmLogger, 5);
car.subscribe(car.THROTTLE_POSITION, throttleLogger, 5);
car.subscribe(car.INTAKE_AIR_TEMPERATURE, airTempLogger, 5);
car.subscribe(car.COOLANT_TEMPERATURE, coolantTempLogger, 5);
car.subscribe(car.FUEL_PRESSURE, fuelPressureLogger, 5);

uart <- hardware.uartFG;
fix <- hardware.pinH;
en <- hardware.pinJ;


gps <- PA6H(uart, en, fix);

gps.setModeRMC();
gps.setUpdateRate(5.0);
gps.setReportingRate(5.0);

i2c <- hardware.i2cAB;
i2c.configure(CLOCK_SPEED_400_KHZ);
imu <- LSM9DS0(i2c);

// Enable the gyro in all axes
imu.setPowerState_G(1);
// Enable Accelerometer in all axes
imu.setEnable_A(1);
// Set the Accelerometer data rate to a nonzero value
imu.setDatarate_A(10); // 10 Hz

function log_stuff() {
    gpsLog = {"msg": gps.getPosition()};
    gpsLogger(gpsLog);
    local acc = imu.getAccel();
    accelLogger({"msg": acc});
    timeLogger({"msg": time()});
    imp.wakeup(5, log_stuff);
}

log_stuff();
