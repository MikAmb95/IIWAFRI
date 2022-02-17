/**

DISCLAIMER OF WARRANTY

The Software is provided "AS IS" and "WITH ALL FAULTS," 
without warranty of any kind, including without limitation the warranties 
of merchantability, fitness for a particular purpose and non-infringement. 
KUKA makes no warranty that the Software is free of defects or is suitable 
for any particular purpose. In no event shall KUKA be responsible for loss 
or damages arising from the installation or use of the Software, 
including but not limited to any indirect, punitive, special, incidental 
or consequential damages of any character including, without limitation, 
damages for loss of goodwill, work stoppage, computer failure or malfunction, 
or any and all other commercial damages or losses. 
The entire risk to the quality and performance of the Software is not borne by KUKA. 
Should the Software prove defective, KUKA is not liable for the entire cost 
of any service and repair.


COPYRIGHT

All Rights Reserved
Copyright (C)  2014-2015 
KUKA Roboter GmbH
Augsburg, Germany

This material is the exclusive property of KUKA Roboter GmbH and must be returned 
to KUKA Roboter GmbH immediately upon request.  
This material and the information illustrated or contained herein may not be used, 
reproduced, stored in a retrieval system, or transmitted in whole 
or in part in any way - electronic, mechanical, photocopying, recording, 
or otherwise, without the prior written consent of KUKA Roboter GmbH.  






\file
\version {1.9}
*/
#include <stdlib.h>
#include <stdio.h>
#include <string.h> // strstr
#include <iostream>
#include <sstream>
#include <fstream>
#include "LBRTorqueSineOverlayClient.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"
#include <unistd.h>
#include <mutex>
#include <boost/thread/thread.hpp>




using namespace KUKA::FRI;
using namespace std;

#define SERVER "192.170.10.2"
#define CLIENT "192.170.10.146"
#define DEFAULT_PORTID 30200
#define DEFAULT_JOINTMASK 0x8
#define DEFAULT_FREQUENCY 0.25
#define DEFAULT_AMPLITUDE 15.0



ROBOT_STATE rc;
std::mutex rc_mtx;


void get_command() {

   cout<<"HERE"<<endl;
   int input_socket;
   listener_socket(9031, &input_socket);

   int slen, rlen;
	sockaddr_in si_other;
	ROBOT_STATE rc_in;
	while( true ) {
		rlen = recvfrom( input_socket, &rc_in, sizeof(rc_in),0,(struct sockaddr*)&si_other, (socklen_t*)&slen);
      if (rlen>0) {
         rc_mtx.lock();
         for(int i=0; i<7; i++ )
            rc.jstate[i] = rc_in.jstate[i];
         rc_mtx.unlock();

         //cout << "Jcommand: ";
         //for(int i=0; i<7; i++ ) cout << rc.jcmd[i] << " ";
         //cout << endl;
      }
	}
}



int main (int argc, char** argv)
{

   ROBOT_STATE rs;
   ROBOT_STATE trq_in;
   // parse command line arguments
   if (argc > 1)
   {
	   if ( strstr (argv[1],"help") != NULL)
	   {
	      printf(
	            "\nKUKA LBR torque sine overlay test application\n\n"
	            "\tCommand line arguments:\n"
	            "\t1) remote hostname (optional)\n"
	            "\t2) port ID (optional)\n"
	            "\t3) bit mask encoding of joints to be overlaid (optional)\n"
	            "\t4) sine frequency in Hertz (optional)\n"
	            "\t5) sine amplitude in Nm (optional)\n"
	      );
	      return 1;
	   }
   }
   char* hostname = (argc >= 2) ? argv[1] : NULL;
   int port = (argc >= 3) ? atoi(argv[2]) : DEFAULT_PORTID;
   unsigned int jointMask = (argc >= 4) ? (unsigned int)atoi(argv[3]) : DEFAULT_JOINTMASK;
   double frequency = (argc >= 5) ? atof(argv[4]) : DEFAULT_FREQUENCY;
   double amplitude = (argc >= 6) ? atof(argv[5]) : DEFAULT_AMPLITUDE;
  
   int out_socket;

   boost::thread func( &get_command );

   create_socket( CLIENT, 9030, &out_socket);
   /***************************************************************************/
   /*                                                                         */
   /*   Place user Client Code here                                           */
   /*                                                                         */
   /**************************************************************************/
   
   // create new sine overlay client
   LBRTorqueSineOverlayClient client(jointMask, frequency, amplitude);

   /***************************************************************************/
   /*                                                                         */
   /*   Standard application structure                                        */
   /*   Configuration                                                         */
   /*                                                                         */
   /***************************************************************************/

   // create new udp connection
   UdpConnection connection;


   // pass connection and client to a new FRI client application
   ClientApplication app(connection, client);
   
   bool success = true;
   // connect client application to KUKA Sunrise controller
   success=app.connect(30200, SERVER);
   printf("Success: %d\n", success);
   

   /***************************************************************************/
   /*                                                                         */
   /*   Standard application structure                                        */
   /*   Execution mainloop                                                    */
   /*                                                                         */
   /***************************************************************************/

   // repeatedly call the step routine to receive and process FRI packets

  ofstream myfile_trq;
myfile_trq.open("/home/utente/ros_ws/src/iiwa_kdl/src/trq.m");
myfile_trq<<"trq = [";
  
   while (success)
   {  

      
      

      client.get_js( rs );
      write( out_socket, &rs, sizeof(rs) );
      
      //Apply command ( read from socket ) 
      rc_mtx.lock();
      client.set_trq(rc);
      rc_mtx.unlock();
      client.get_trq (trq_in);
      cout<<"TEST: "<<trq_in.jstate[1]<<endl;
     
     if(rs.jstate[0] < 29.9*3.14/180){
     myfile_trq<<trq_in.jstate[0]<<" ";
     myfile_trq<<trq_in.jstate[1]<<" ";
     myfile_trq<<trq_in.jstate[2]<<" ";
     myfile_trq<<trq_in.jstate[3]<<" ";
     myfile_trq<<trq_in.jstate[4]<<" ";
     myfile_trq<<trq_in.jstate[5]<<" ";
     myfile_trq<<trq_in.jstate[6]<<" ";
     myfile_trq<<"\n";
     }
     else {
     cout<<"---------------------- END ---------------"<<endl;
     myfile_trq<<trq_in.jstate[0]<<" ";
     myfile_trq<<trq_in.jstate[1]<<" ";
     myfile_trq<<trq_in.jstate[2]<<" ";
     myfile_trq<<trq_in.jstate[3]<<" ";
     myfile_trq<<trq_in.jstate[4]<<" ";
     myfile_trq<<trq_in.jstate[5]<<" ";
     myfile_trq<<trq_in.jstate[6]<<" ";
     myfile_trq<<"]; \n";
     //fprintf(testo,"%f]; \n",trq_in.jstate[6]);

      myfile_trq.close();
     }
            

      success = app.step();
   }

   /***************************************************************************/
   /*                                                                         */
   /*   Standard application structure                                        */
   /*   Dispose                                                               */
   /*                                                                         */
   /***************************************************************************/

   // disconnect from controller
   app.disconnect();
   
   return 1;
}
