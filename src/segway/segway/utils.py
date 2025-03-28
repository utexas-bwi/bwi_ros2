"""--------------------------------------------------------------------
COPYRIGHT 2014 Stanley Innovation Inc.

Software License Agreement:

The software supplied herewith by Stanley Innovation Inc. (the "Company") 
for its licensed Segway RMP Robotic Platforms is intended and supplied to you, 
the Company's customer, for use solely and exclusively with Stanley Innovation 
products. The software is owned by the Company and/or its supplier, and is 
protected under applicable copyright laws.  All rights are reserved. Any use in 
violation of the foregoing restrictions may subject the user to criminal 
sanctions under applicable laws, as well as to civil liability for the 
breach of the terms and conditions of this license. The Company may 
immediately terminate this Agreement upon your use of the software with 
any products that are not Stanley Innovation products.

The software was written using Python programming language.  Your use 
of the software is therefore subject to the terms and conditions of the 
OSI- approved open source license viewable at http://www.python.org/.  
You are solely responsible for ensuring your compliance with the Python 
open source license.

You shall indemnify, defend and hold the Company harmless from any claims, 
demands, liabilities or expenses, including reasonable attorneys fees, incurred 
by the Company as a result of any claim or proceeding against the Company 
arising out of or based upon: 

(i) The combination, operation or use of the software by you with any hardware, 
    products, programs or data not supplied or approved in writing by the Company, 
    if such claim or proceeding would have been avoided but for such combination, 
    operation or use.
 
(ii) The modification of the software by or on behalf of you 

(iii) Your use of the software.

 THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 
 \file   utils.py

 \brief  This module contains general utility functions

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
import struct
import socket
import math
from .crc16 import compute_buffer_crc, buffer_crc_is_valid
import array
from .system_defines import *

"""
slew limit funtion to limit the maximum rate of change
"""
def slew_limit(signal_in,signal_out,max_rate,dt):
    if (0 == dt):
        return
    requested_rate = (signal_in - signal_out)/dt
                
    if (requested_rate > max_rate): 
        signal_out += max_rate * dt
    elif (requested_rate >= -max_rate): 
        signal_out = signal_in
    else: 
        signal_out += -max_rate * dt
    
    return signal_out


"""
Make a 16-bit value from two 8-bit values
"""
def m16(byte_array):

    return (( byte_array[0] << 8) & 0xFF00) | (byte_array[1] & 0x00FF)

"""
Make a 32-bit value from four 8-bit values
"""
def m32(byte_array):
    ret = 0;
    ret |= (byte_array[0] & 0xFF) << 24
    ret |= (byte_array[1] & 0xFF) << 16
    ret |= (byte_array[2] & 0xFF) << 8
    ret |= (byte_array[3] & 0xFF)

    return ret

def generate_cmd_bytes(cmd):
    cmd_bytes = []
    add_bytes(cmd_bytes,cmd[0],16)
    for cmd_var in cmd[1]:
        add_bytes(cmd_bytes,cmd_var,32)
    
    """
    Generate the CRC for the command bytes
    """
    compute_buffer_crc(cmd_bytes)

    return cmd_bytes
    
def  validate_response(rsp):
    
    """
    Check the CRC and 
    """
    rsp = bytes(rsp)
    data = array.array('B',rsp)
    data = array.array('I',data.tobytes())
    final_data = data
    rsp = array.array('B',data.tobytes())
    if (buffer_crc_is_valid(rsp)) and (len(rsp) > 0):
        return True,final_data[:len(final_data)-1]
       
    """
    Not a valid CRC
    """
    return False,None
    
def add_bytes(list_to_append,var2,bits):
    if bits % 2:
        return False
    
    bytes_to_make = bits//8
    for i in range(0,bytes_to_make):
        shift = bits - 8*(i+1)
        list_to_append.append((var2 >> shift) & 0xFF)    

"""
For IEEE754 processors this function converts a 32-bit floating point number to
a 32-bit integer representation
"""
def convert_float_to_u32(value):
    return struct.unpack('=I', struct.pack('=f', value))[0]

"""
For IEEE754 processors this function converts a 32-bit integer representation
of a floating point value to float representation
"""
def convert_u32_to_float(bits):
    return struct.unpack('=f', struct.pack('=I', bits))[0]

def convert_u64_to_double(high_word,low_word):
    temp = (high_word << 32) & 0xFFFFFFFF00000000
    temp |= (low_word & 0x00000000FFFFFFFF)
    
    return struct.unpack('=d', struct.pack('=Q', temp))[0]

"""
Used to convert a byte array (string) into an array of 32-bit values
"""
def convert_byte_data_to_U32(data):

    rx_dat = [];
    k = 0;
    
    #
    # Convert the string into a byte array
    #
    
    for x in range(0,len(data)):
        rx_dat.append(ord(data[x]));
        
    number_of_u32s = (len(rx_dat)/4)

    #
    # Convert the byte array into an array of 32bit values
    #
    converted = [0]*number_of_u32s;
    for x in range(0,number_of_u32s):
        converted[x] = int((((rx_dat[k]   << 24) & 0xFF000000)) |
                        (((rx_dat[k+1] << 16) & 0x00FF0000)) |
                        (((rx_dat[k+2] << 8)  & 0x0000FF00)) |
                          (rx_dat[k+3] & 0x000000FF));

        k+=4;
        
    return converted;

"""
Used to convert an IP address string in dotted quad format to an integer
"""  
def dottedQuadToNum(ip):
    "convert decimal dotted quad string to long integer"
    return struct.unpack('I',socket.inet_aton(ip))[0]

"""
Used to convert an IP address in integer format to a dotted quad string
""" 
def numToDottedQuad(n):
    "convert long int to dotted quad string"
    return socket.inet_ntoa(struct.pack('I',n))

def limit_f(signal_in, limit):
    
    if (signal_in > abs(limit)):
        return abs(limit)
    elif (signal_in <= -abs(limit)):
        return -abs(limit)
    else:
        return signal_in
    
def clamp_value_f(value,lower_limit,upper_limit):
    
    if (value < lower_limit):
        value = lower_limit;
    elif (value > upper_limit):
        value = upper_limit;
    
    return value;

def minimum_f(input1,input2):
    
    if (math.fabs(input1) > math.fabs(input2)):
        return input2
    return input1

def approx_equal(in_1,in_2,max_delta):
    
    if abs(in_1 - in_2) <= max_delta :
        return True
    return False




