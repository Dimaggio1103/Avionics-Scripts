#! /usr/bin/env python

import time
import struct
import socket
import sys
import math
import errno
from optparse import OptionParser, OptionValueError

#keep these here? put in class?
IOF_GPS_TIME_MARK_INFO = 1024
IOF_ARINC_SPARE_RX3_ARR = 1029
IOF_A429_ALTITUDE_RATE = 1065
IOF_SS_ALTITUDE_RATE = 1096
IOF_SS_HEADING = 1101
IOF_SS_ALTITUDE = 1179
IOF_GPS_IOP_STATUS = 10118
IOF_MPC2_GPS_CHANNEL_STATUS = 10113
IOF_FMS_MAGNETIC_VARIATION = 805
IOF_SHADIN_ALTITUDE = 1055
IOF_A429_BARO_UNCORR_ALTITUDE = 1057
IOF_A429_BARRO_CORR_ALTITUDE = 1058
IOF_A429_TRUE_AIR_SPEED = 1060
IOF_A429_COMPUTED_AIR_SPEED = 1064
IOF_MMDS_VOR_ILS_STATION_ID = 1087

IOF_SS_TRUE_AIRSPEED = 1100
IOF_TRAFFIC_LABELS_RX = 1183

IOF_AHRS_HEADING_ANGLE  = 1042
IOF_AHRS_TRUE_HEADING = 1044

IOF_MPC1_IOF_CONTROL = 0xFFFF
IOF_MPC2_IOF_CONTROL = 0xFFFE

#time variables
IOF_MPC2_GNSS_TX = 2206
IOF_MPC2_GNSS_RX = 2207
IOF_MPC2_GNSS_CNTL = 2208
IOF_FMS_RTC_DATE_TIME = 948

fifty_ms = 50/1000.0
one_hundred_ms = 100/1000.0
two_hundred_ms = 200/1000.0
five_hundred_ms = 500/1000.0
TASK_INTERVAL = fifty_ms
CONTROL_START = 0
CONTROL_INTERVAL = 1
TIME_EPSILON = 10/1000.0

GPS_START = fifty_ms
GPS_INTERVAL = two_hundred_ms

ADC_START = one_hundred_ms
ADC_INTERVAL = two_hundred_ms

AHRS_START = one_hundred_ms + fifty_ms
AHRS_INTERVAL = two_hundred_ms

FMS_RTC_START = two_hundred_ms
FMS_RTC_INTERVAL = five_hundred_ms

class ksn_send(object):
    def __init__(self, host, port):
        '''Constructor'''
        self.host = host
        self.port = port
        self.sock = socket.socket(type=socket.SOCK_DGRAM)
    
    
    def __send(self, package):
        '''Internal socket wrapper'''
        try:
            self.sock.sendto(package, (self.host, self.port))
        except socket.error, (value,message):
            if value ==  errno.EHOSTDOWN or value == errno.EHOSTUNREACH:
                pass
            else:
                self.close()
                raise
        return
    
    
    def _send_iof_command(self, command, block_magvar, block_ias,
                         block_true_heading, block_time, block_vor_id):
        '''Block certain labels'''
        label = struct.pack("!H", IOF_MPC2_IOF_CONTROL)
        str_format = "\0%d-%d"
        str_format += "\0%d"*2
        values = [  IOF_GPS_TIME_MARK_INFO, IOF_A429_ALTITUDE_RATE,
                    IOF_GPS_IOP_STATUS,
                    IOF_MPC2_GPS_CHANNEL_STATUS]
        if block_magvar:
            values.append(IOF_FMS_MAGNETIC_VARIATION)
            str_format += "\0%d"
        if block_ias:
            values.append(IOF_A429_COMPUTED_AIR_SPEED)
            str_format += "\0%d"
        if block_true_heading:
            values.append(IOF_AHRS_TRUE_HEADING)
            str_format += "\0%d"
        if block_time:
            values.append(IOF_FMS_RTC_DATE_TIME)
            str_format += "\0%d"
        if block_vor_id:
            values.append(IOF_MMDS_VOR_ILS_STATION_ID)
            str_format += "\0%d"
        '''
        Traffic could be blocked, but this script is probably not needed if
        traffic is already being sent to the unit
        '''
        formatted_values = str_format % tuple(values)
        package = label + command + formatted_values
        self.__send(package)
    
    
    def send_block_iof( self, block_magvar, block_ias, block_true_heading,
                        block_time, block_vor_id):
        '''block the iof on certain labels to prevent writes on them'''
        self._send_iof_command("block_write", block_magvar, block_ias,
                               block_true_heading, block_time, block_vor_id)
    
    
    def send_unblock_iof( self, block_magvar, block_ias, block_true_heading,
                        block_time, block_vor_id):
        '''unblock the iof for the blocked labels'''
        self._send_iof_command("unblock_write", block_magvar, block_ias,
                               block_true_heading, block_time, block_vor_id)
    
    
    def send_pxpress_3a(self):
        '''To keep fms happy'''
        package = struct.pack("!HxBB", IOF_MPC2_GPS_CHANNEL_STATUS, 42, 6)
        for i in range(6):
            package += struct.pack("!BxBxxxxxxx", i+1, 0x04)
        self.__send(package)
    
    
    def send_gps_time_mark( self, latitude, longitude, altitude, ground_track,
                            ground_speed, vertical_speed):
        '''Send GPS info and current date/time'''
        label = IOF_GPS_TIME_MARK_INFO
        
        current_time = time.localtime()
        year = current_time.tm_year
        month = current_time.tm_mon
        day = current_time.tm_mday
        hour = current_time.tm_hour
        minute = current_time.tm_min
        second = current_time.tm_sec
        
        package = struct.pack(  "!HddffffHBBBBBx", label, dec_to_rad(latitude), 
                                dec_to_rad(longitude), ft_to_m(altitude),
                                ground_track, kts_to_mps(ground_speed),
                                ftpm_to_mps(vertical_speed), year, month, day,
                                hour, minute, second)
        self.__send(package)
    
    
    def send_pxpress_31(self):
        '''To keep FMS happy'''
        label = IOF_GPS_IOP_STATUS
        mode = 0x02
        hdop = vdop = 1
        hpe = vpe = 555
        hil = vil = 100

        format = "!HxB""x""x""x""xxxx""ffffff""x""x""xx""x""xxxx""xxxx""xx""x"
        
        package = struct.pack(  format, label, mode, hdop, hil, hpe, vdop, 
                                vil, vpe)
        self.__send(package)
    
    
    def send_shadin_barro_uncorr_alt(self, altitude):
        '''end uncorrected shadin altitude'''
        label = IOF_SHADIN_ALTITUDE
        #data1 is valid, data2 is invalid
        package = struct.pack("!HBxxxfBxxxfxxxx", label, 1, altitude, 0, 0)
        self.__send(package)
    
    
    def send_a429_barro_uncorr_alt(self, altitude):
        '''Send uncorrected ARINC 429 altitude'''
        label = IOF_A429_BARO_UNCORR_ALTITUDE
        package = struct.pack("!HBxxxfxxxxxxxx", label, 1, altitude)
        self.__send(package)
    
    
    def send_a429_barro_corr_alt(self, altitude):
        '''Send corrected ARINC 429 altitude'''
        label = IOF_A429_BARRO_CORR_ALTITUDE
        package = struct.pack("!HBxxxfxxxxxxxx", label, 1, altitude)
        self.__send(package)
    
    
    def send_a429_true_airspeed(self, tas):
        '''send TAS'''
        label = IOF_A429_TRUE_AIR_SPEED
        package = struct.pack("!HBxxxfxxxx", label, 1, tas)
        self.__send(package)
    
    
    def send_ahrs_mag_heading_angle(self, angle):
        '''send mag heading'''
        label = IOF_AHRS_HEADING_ANGLE
        package = struct.pack("!HBxxxfxxxx", label, 1, angle)
        self.__send(package)
    
    
    def send_ahrs_true_heading_angle(self, angle):
        #send true heading
        label = IOF_AHRS_TRUE_HEADING
        package = struct.pack("!HBxxxfxxxx", label, 1, angle)
        self.__send(package)    
    
    
    def send_magvar(self, magvar):
        '''send magnetic variation'''
        label = IOF_FMS_MAGNETIC_VARIATION
        package = struct.pack("!Hf", label, magvar)
        self.__send(package)


    def send_ias(self, ias):
        '''send indicated airspeed'''
        label = IOF_A429_COMPUTED_AIR_SPEED
        package = struct.pack("!HBxxxfxxxx", label, 1, ias)
        self.__send(package)
    
    def _send_basic_trfc_packet(self, discrete_label, intruder_type):
        '''send a basic arinc packet.  Suitable for standby, unavailable, coast,
        and removed pop-ups.'''
        if intruder_type < 0:
            intruder_type = 0
        elif intruder_type > 3:
            intruder_type = 3
        
        label = IOF_TRAFFIC_LABELS_RX
        arinc_rts = 0x120005EF # 5 count
        arinc_etx = 0x030005EF # 5 count
        intruder_range = 0x62808058
        intruder_altitude = 0x62908059
        intruder_bearing = 0x6400005A #bits 16, 17, 18 determine type
        intruder_bearing |= (intruder_type << 15)
        package = struct.pack("!HIIIIII", label, discrete_label, arinc_rts,
                              intruder_range, intruder_altitude,
                              intruder_bearing, arinc_etx)
        self.__send(package)
    
    
    def send_trfc_duplicates(self):
        intruder_type = 0
        label = IOF_TRAFFIC_LABELS_RX
        discrete_label = 0x880000bc
        arinc_rts = 0x120008EF # 8 count
        arinc_etx = 0x030008EF # 8 count
        intruder_range = 0x62808058
        intruder_altitude = 0x62908059
        intruder_bearing = 0x6400005A #bits 16, 17, 18 determine type
        intruder_bearing |= (intruder_type << 15)
        package = struct.pack("!HIIIIIIIII", label, discrete_label, arinc_rts,
                              intruder_range, intruder_altitude, 
                              intruder_bearing, intruder_range,
                              intruder_altitude, intruder_bearing, arinc_etx)
        self.__send(package)
    
    
    def send_trfc_comp_unit(self):
        label = IOF_TRAFFIC_LABELS_RX
        arinc_0350 = 0x000000e8
        TCAS_COMPUTER_UNIT = 0x00000200
        arinc_0350 |= TCAS_COMPUTER_UNIT
        self._send_basic_trfc_packet(arinc_0350, 0)
    
    def send_trfc_unavailable(self, intruder_type):
        '''
        creates an unavailable annunciation
        '''
        arinc_0274 = 0x000000bc
        self._send_basic_trfc_packet(arinc_0274, intruder_type)
    
    
    def send_trfc_standby(self, intruder_type):
        '''
        creates a stanby annunciaiton
        '''
        arinc_0274 = 0x010000bc
        self._send_basic_trfc_packet(arinc_0274, intruder_type)
    
    
    def send_trfc_operating(self, intruder_type):
        '''
        displays a single intruder with no special annunciation, i.e. normal
        operations
        '''
        arinc_0274 = 0x880000bc
        self._send_basic_trfc_packet(arinc_0274, intruder_type)
        
    
    def send_trfc_coast(self, age, intruder_type):
        '''
        creates a coast annunciation (age <= 12 then coast.  Greater than 12 is
        a removed annunciation) 2**13 is max age
        '''
        if age < 0:
            age = 0
        
        if age >= 8192:
            age = 8191
            
        age_offset = 8 #age starts at the ninth bit and goes to the 21st
        arinc_0274 = 0x020000bc | (age << age_offset)
        self._send_basic_trfc_packet(arinc_0274, intruder_type)
    
        
    def send_trfc_test(self, intruder_type):
        '''Creates a test annunciation'''
        arinc_016 = 0x4100000e
        self._send_basic_trfc_packet(arinc_016, intruder_type)
        
        
    def send_TAWS_warning_popup(self):
        '''creates a TAWS warning popup'''
        label = IOF_ARINC_SPARE_RX3_ARR
        arinc_0274 = 0x00000010bc # GP_TAWS_ANNUN_PULL_UP should be set in 274
        package = struct.pack("!HI", label, arinc_0274)
        self.__send(package)
    
    
    def send_TAWS_caution_popup(self):
        '''creates a TAWS caution popup'''
        label = IOF_ARINC_SPARE_RX3_ARR
        arinc_0274 = 0x00000008bc # GP_TAWS_ANNUN_GND_PROX should be set in 274
        package = struct.pack("!HI", label, arinc_0274)
        self.__send(package)
    
    
    def send_TAWS_clear_inhibit_popup(self):
        '''clears TAWS popup inhibition (needs to be done to be able to send
        another TAWS popup)'''
        label = IOF_ARINC_SPARE_RX3_ARR
        arinc_0274 = 0x00000000bc # Blank 274 word
        package = struct.pack("!HI", label, arinc_0274)
        self.__send(package)
    
   
    def send_fms_rtc(self, year, month, day, hours, minutes, seconds):
        '''sends fms real time clock information'''
        label = IOF_FMS_RTC_DATE_TIME
        package = struct.pack("!HBBBBBBH", label, seconds, minutes, hours, day,
                              month, 1, year)
        self.__send(package)
        
    def send_vor_id(self, id):
        '''sends VOR ILS station id'''
        label = IOF_MMDS_VOR_ILS_STATION_ID
        id = id[:4] #limit to 4
        while len(id) < 4:
            id += '\0'
        id = id[::-1] #reverse
        package = struct.pack("!Hcccc", label, id[0], id[1], id[2], id[3])
        self.__send(package)
    
    
    def close(self):
        '''closes the socket'''
        self.sock.close()


def handle_rtc_time(option, opt_str, value, parser):
    #order is (year, month, day, hour, min, sec)
    year = value[0]
    month = value[1]
    day = value[2]
    hour = value[3]
    min = value[4]
    sec = value[5]
    if year < 2000:
        error = "option %s: year must be >= to 2000 (got: %d)" % (opt_str, year)
        raise OptionValueError(error)
    
    if month < 1 or month > 12:
        error = "option %s: month must be in [1,12] (got %d)" % (opt_str, month)
        raise OptionValueError(error)
    
    if day < 1 or day > 31:
        error = "option %s: day must in [1, 31] (got %d)" % (opt_str, day)
        raise OptionValueError(error)
    
    if hour < 0 or hour > 23:
        error = "option %s: hour must in [0, 23] (got %d)" % (opt_str, hour)
        raise OptionValueError(error)
    
    if min < 0 or min > 59:
        error = "option %s: minutes must be in [0,59] (got %d)" % (opt_str, min)
        raise OptionValueError(error)
    
    if sec < 0 or sec > 59:
        error = "option %s: seconds must be in [0,59] (got %d)" % (opt_str, sec)
        raise OptionValueError(error)
    
    parser.values.fms_rtc_year = year
    parser.values.fms_rtc_month = month
    parser.values.fms_rtc_day = day
    parser.values.fms_rtc_hour = hour
    parser.values.fms_rtc_min = min
    parser.values.fms_rtc_sec = sec
    return


def handle_intruder_type(option, opt_str, value, parser):
    #nt = 0, ta = 1, ra = 2, pa = 3
    if value == "nt":
        parser.values.intruder_type = 0
    elif value == "ta":
        parser.values.intruder_type = 1
    elif value == "ra":
        parser.values.intruder_type = 2
    else:
        parser.values.intruder_type = 3
    return


def setup_command_line():
    usage = """usage: %prog [options]
    
    This sends the information found in the options to the host:port.  The
    default value for each option is 0, unless otherwise specified.
    
    Use Ctrl-C to exit out of this script once running.
    """;
    #better defaults?
    parser = OptionParser(usage=usage)
    
    help = "By default, the message is sent to the external KSN unit. To send "
    help += "to the mac simulator, the host should be 127.0.0.1 ."
    help += "[default:%default]"
    parser.add_option(  "--host", action="store", dest="host", help=help,
                        default="192.168.28.32", type="string");
    help = "[default:%default]"
    parser.add_option(  "--port", action="store", dest="port", help=help,
                        default=3471, type="int");

    help = "Time value sent by the FMS.  The format is --fmstime year month day "
    help += "hour minute second.  NOTE: LEADING ZEROS CAUSE THE NUMBER TO BE "
    help += "TREATED AS OCTAL NUMBERS.  Not sent by default."
    parser.add_option(  "--fmstime", action="callback", callback=handle_rtc_time,
                        help=help, type="int", dest="fms_rtc_sec",nargs=6);

    help = "Timeout of the script (in seconds).  If not set, the script will run"
    help += " until the keyboard interrupt signal (Ctrl-C) is given."
    parser.add_option(  "--timeout", action="store", dest="timeout", help=help,
                        type="int");
    help = "In degrees [default:%default]"
    parser.add_option(  "--latitude", action="store", dest="lat", help=help,
                        default=35.0403, type="float");
    parser.add_option(  "--longitude", action="store", dest="lon", help=help,
                        default=-106.6092, type="float");
    help = "In feet [default:%default]"
    parser.add_option(  "--altitude", action="store", dest="alt", help=help,
                        default=5355, type="float");
    help = "In knots"
    parser.add_option(  "--groundspeed", action="store", help=help,
                        dest="ground_speed", default=0, type="float");
    parser.add_option(  "--groundtrack", action="store", 
                        dest="ground_track", default=0, type="float");
    help = "In feet per minute"
    parser.add_option(  "--verticalspeed", action="store", help=help,
                        dest="vertical_speed", default=0, type="float");
    help = "[default:%default]"
    parser.add_option(  "--barrocorralt", action="store", help=help,
                        dest = "barro_corr_alt", default=5355, type="float");
    help = "No default value.  Also turns off sending of barrocorralt if "
    help += "specified"
    parser.add_option(  "--barrouncorralt", action="store", help=help,
                        dest = "barro_uncorr_alt", type="float");
    parser.add_option(  "--trueairspeed", action="store",
                        dest = "true_airspeed", default=0, type="float");
    parser.add_option(  "--magheading", action="store", dest="magheading",
                        default=0, type="float");
    help = "No default value.  Needs to have shadin format C set.  Needs to "
    help += "have arinc sources not 0 (PREF2_NUMADS) or serial shadin format"
    help += "(PREF2_SERIAL_SHADIN_FORMAT) to 2 ."
    parser.add_option(  "--shadinuncorralt", action="store",
                        dest="shadin_uncorr_alt", help=help, type="float");
    help = "No default value (true heading does not get sent unless it is "
    help += "specified)."
    parser.add_option(  "--trueheading", action="store", dest="trueheading",
                        help=help, type="float");
    help = "No default value (magvar does not get sent unless it is specified). "
    help += "FMS sends its own magnetic variation, so you should not need to "
    help += "send it when FMS is working (i.e. on the KSN unit)."
    parser.add_option(  "--magvar", action="store", dest="magvar", help=help,
                        type="float");
    help = "No default value (indicated airspeed does not get sent unless it is "
    help += "specified)."
    parser.add_option(  "--ias", action="store", dest="ias", help=help,
                        type="float");
    help = "Traffic types and annunciations.  Possible values are 'n' for normal "
    help += "operations, 's' for standby, 'u' for unavailable, 't' for test, "
    help += "'d' for sending duplicate intruders, 'f' for the Comp Unit "
    help += "failure, and 'c' for coast.  Coast has  the additional coast age "
    help += "option (from which a removed annunciation can be created). One "
    help += "traffic advisory intruder is sent.  No traffic data is sent by "
    help += "default."
    choices = ["s", "u", "c", "t", "n", "d", "f"]
    parser.add_option(  "--traffictype", action="store", dest="traffic_type",
                        help=help, type="choice", choices=choices);
    
    help = "The intruder type for traffic.  'nt' is 'no threat', 'ta' is "
    help += "'traffic advisory', 'ra' is 'resolution advisory', and 'pa' is "
    help += "'proximity advisory'. Default is 'nt' (i.e. no threat)."
    choices = ["nt", "ta", "ra", "pa"]
    parser.add_option("--intrudertype", action="callback",
                      callback=handle_intruder_type, dest="intruder_type",
                      help=help, type="choice", choices=choices, default=0)
    
    help = "Age of the coast annunciation.  Ages > 12 changes the coast "
    help += "annunciation to a removed annunciation.  Ages should be greater "
    help += "than or equal to 0 and less than 8192.  Option is ignored if the "
    help += "traffictype is not coast. [default:%default]"
    parser.add_option(  "--coastage", action="store", dest="coast_age",
                        help=help, type="int", default=0);
    
    
    help = "The TAWS popup type.  'w' is 'warning' and 'c' is 'caution'.  "
    help += "Need to stop the script and clean up to be able to send another "
    help += "popup."
    choices = ["w", "c"]
    parser.add_option("--tawspopup", action="store", dest="taws_popup",
                      help=help, type="choice", choices=choices)
    
    help = "The VOR ILS station id.  Not sent by default. Note: This will make "
    help = "the id blink on the simulator since the simulator self-sends the id."
    parser.add_option("--ident", action="store", dest="vor_id", help=help)
    
    help = "If true, the script does not send the unblock command at the end. "
    help += "[default:%default]"
    parser.add_option(  "--nocleanup", action="store_true", dest="nocleanup",
                        help=help, default=False);
    return parser


def dec_to_rad(value):
    return value * math.pi / 180


def ft_to_m(value):
    FT_TO_M = 0.30484
    return value * FT_TO_M


def kts_to_mps(value):
    KTS_TO_MPS = 0.51444445
    return value * KTS_TO_MPS


def ftpm_to_mps(value):
    FTPM_TO_MPS = 0.00508
    return value * FTPM_TO_MPS


def valid_coast_age(age):
    if age >= 0 and age < 2**13:
        return True
    else:
        return False


def main():
    parser = setup_command_line()
    (options, args) = parser.parse_args()
    
    traffic_type_set = False
    if options.traffic_type != None:
        traffic_type_set = True
        if options.traffic_type == "c" and not valid_coast_age(options.coast_age):
            parser.error("coastage must be greater than or equal to 0 and less than 8192.")
    
    timeout_set = False
    if options.timeout != None:
        timeout_set = True
        if options.timeout < 0:
            parser.error("timeout must be greater than or equal to 0")
    
    time_set = options.fms_rtc_sec != None
    magvar_set = options.magvar != None
    ias_set = options.ias != None
    true_heading_set = options.trueheading != None
    barro_uncorr_alt_set = options.barro_uncorr_alt != None
    shadin_uncorr_alt_set = options.shadin_uncorr_alt != None
    vor_id_set = options.vor_id != None
    
    #don't need the parser anymore
    parser.destroy()
    
    ksnsend = ksn_send(host=options.host, port=options.port)
    
    #set up times
    ctrl_next_time = CONTROL_START - TIME_EPSILON
    gps_next_time = GPS_START - TIME_EPSILON
    adc_next_time = ADC_START - TIME_EPSILON
    ahrs_next_time = AHRS_START - TIME_EPSILON
    fms_rtc_next_time = FMS_RTC_START - TIME_EPSILON
    end_time = None
    if timeout_set:
        print "Sending data for %d seconds" % options.timeout
    print "Terminate with Ctrl-Break on Win32, Ctr-C on Unix"
    print "(Run with the -h or --help option for further help)"
    try:
        
        while True:
            now = time.time()
            if timeout_set:
                if end_time == None:
                    end_time = now + options.timeout
                
                if now > end_time:
                    break
            
            if now >= ctrl_next_time:
                ctrl_nex_time = now + CONTROL_INTERVAL - TIME_EPSILON
                
                ksnsend.send_block_iof(magvar_set, ias_set, true_heading_set,
                                       time_set, vor_id_set)
                
                ksnsend.send_pxpress_3a()
                if magvar_set:
                    ksnsend.send_magvar(options.magvar)
                if ias_set:
                    ksnsend.send_ias(options.ias);
            
            if now >= gps_next_time:
                gps_next_time = now + GPS_INTERVAL - TIME_EPSILON
                ksnsend.send_gps_time_mark(options.lat, options.lon, 
                                        options.alt, options.ground_track,
                                        options.ground_speed, 
                                        options.vertical_speed)
                ksnsend.send_pxpress_31()
            
            if now >= adc_next_time:
                adc_next_time = now + ADC_INTERVAL - TIME_EPSILON
                if barro_uncorr_alt_set:
                    ksnsend.send_a429_barro_uncorr_alt(options.barro_uncorr_alt)
                else:
                    ksnsend.send_a429_barro_corr_alt(options.barro_corr_alt)
                ksnsend.send_a429_true_airspeed(options.true_airspeed)
                
                if shadin_uncorr_alt_set:
                    ksnsend.send_shadin_barro_uncorr_alt(options.shadin_uncorr_alt)
                    
                if options.traffic_type == "s":
                    ksnsend.send_trfc_standby(options.intruder_type)
                elif options.traffic_type == "u":
                    ksnsend.send_trfc_unavailable(options.intruder_type)
                elif options.traffic_type == "c":
                    ksnsend.send_trfc_coast(options.coast_age,
                                            options.intruder_type)
                elif options.traffic_type == "t":
                    ksnsend.send_trfc_test(options.intruder_type)
                elif options.traffic_type == "n":
                    ksnsend.send_trfc_operating(options.intruder_type)
                elif options.traffic_type == "d":
                    ksnsend.send_trfc_duplicates()
                elif options.traffic_type == "f":
                    ksnsend.send_trfc_comp_unit()
                
                if options.taws_popup == "w":
                    ksnsend.send_TAWS_warning_popup()
                elif options.taws_popup == "c":
                    ksnsend.send_TAWS_caution_popup()
            
            if now >= ahrs_next_time:
                ahrs_next_time = now + AHRS_INTERVAL - TIME_EPSILON
                if vor_id_set:
                    ksnsend.send_vor_id(options.vor_id)
                ksnsend.send_ahrs_mag_heading_angle(options.magheading)
                if true_heading_set:
                    ksnsend.send_ahrs_true_heading_angle(options.trueheading)

            if now >= fms_rtc_next_time:
                fms_rtc_next_time = now + FMS_RTC_INTERVAL - TIME_EPSILON
                #should go once every half-second (at least that is what the
                #fms does)
                if time_set:
                    ksnsend.send_fms_rtc(options.fms_rtc_year,
                                         options.fms_rtc_month,
                                         options.fms_rtc_day,
                                         options.fms_rtc_hour,
                                         options.fms_rtc_min,
                                         options.fms_rtc_sec)
            
            time.sleep(TASK_INTERVAL)
    except KeyboardInterrupt:
        print ""
        print "Shutting down..."
    except:
        print "Unexpected error:", sys.exc_info()[0]
        raise
    finally:
        if not options.nocleanup:
            print "Cleaning up..."
            #sending a few to try and make sure it gets unblocked
            for i in range(0, 10):
                ksnsend.send_unblock_iof(magvar_set, ias_set, true_heading_set,
                                     time_set, vor_id_set)
                if options.taws_popup != None:
                    ksnsend.send_TAWS_clear_inhibit_popup()
                time.sleep(TASK_INTERVAL)
        ksnsend.close()
        print "Done"
        
if __name__ == "__main__":
    main();