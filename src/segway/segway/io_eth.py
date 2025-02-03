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
 
 \file   udp_socket.py

 \brief  This module contains a threaded ethernet UDP communication driver

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
import select
import socket
import threading
import os
#from bitstring import Bits

class IoEthThread:
    def __init__(self,remote_address,tx_queue,rx_queue,max_packet_size=1500):
        self.tx_queue = tx_queue
        self.rx_queue = rx_queue
        self.max_packet_size = max_packet_size
        self.remote_address = remote_address
        self.listen_terminate_mutex = None
 
        """
        Initialize the UDP connection
        """
        try:
            self.conn = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.conn.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            self.conn.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.conn.settimeout(2)
            self.conn.setblocking(0)
            self.conn.bind(('',self.remote_address[1]))
            self.conn.connect(self.remote_address)
        except:
            try:
                self.conn.close()
            except:
                pass
            self.link_up = False
            return

        self.need_to_terminate = False
        self.listen_terminate_mutex = threading.RLock()
        self.transmit_terminate_mutex = threading.RLock()
        self.listenThread   = threading.Thread(target = self.listen)
        self.transmitThread = threading.Thread(target = self.transmit)
        self.listenThread.start()
        self.transmitThread.start()        
        self.link_up = True
    
    def __del__(self):
        if self.listen_terminate_mutex == None:
            return

        with self.listen_terminate_mutex, self.transmit_terminate_mutex:
            self.need_to_terminate = True
        
        assert(self.listenThread)
        assert(self.transmitThread)
        self.listenThread.join()
        self.transmitThread.join()
        
    def listen(self):
        while True:
            with self.listen_terminate_mutex:
                if self.need_to_terminate:
                    break
            result = select.select([self.conn],[],[],0.1)
            if (len(result[0])>0):
                message = result[0][0].recv(self.max_packet_size)
                message_bytes= list(message)
                self.rx_queue.put(message_bytes)
            
    def transmit(self):
        while True:
            with self.listen_terminate_mutex:
                if self.need_to_terminate:
                    break    
            result = select.select([self.tx_queue._reader],[],[],0.1)
            if (len(result[0])>0):
                data = result[0][0].recv()
                msg_bytes = bytes(data)
                self.conn.send(msg_bytes)

    def Close(self):
        self.__del__()
        self.conn.close()
        self.link_up = False
        
