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
#ifndef _KUKA_FRI_LBR_TORQUE_SINE_OVERLAY_CLIENT_H
#define _KUKA_FRI_LBR_TORQUE_SINE_OVERLAY_CLIENT_H

#include "friLBRClient.h"
#include <iostream>
#include <fcntl.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/mman.h>
#include <sys/io.h>
#include <sys/time.h>
#include <netdb.h>


using namespace KUKA::FRI;

typedef struct ROBOT_STATE {
  double jstate[7];
}ROBOT_STATE;

//Creazione socket in LETTURA
inline bool listener_socket(int port_number, int *sock) {
      sockaddr_in si_me;

  if ( (*sock=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
    std::cout << "Listener::Open: error during socket creation!" << std::endl;
    return false;
  }

  memset((char *) &si_me, 0, sizeof(si_me));

  /* allow connections to any address port */
  si_me.sin_family = AF_INET;
  si_me.sin_port = htons(port_number);
  si_me.sin_addr.s_addr = htonl(INADDR_ANY);
  int bind_ok = bind(*sock, (struct sockaddr*)&si_me, sizeof(si_me));

  if ( bind_ok == -1 )
    return false;
  else
    return true;

}

//Creazione socket in SCRITTURA
inline int create_socket(char* dest, int port, int *sock) {
  struct sockaddr_in temp;
  struct hostent *h;
  int error;

  temp.sin_family=AF_INET;
  temp.sin_port=htons(port);
  h=gethostbyname(dest);

  if (h==0) {
    printf("Gethostbyname fallito\n");
    exit(1);
  }

  bcopy(h->h_addr,&temp.sin_addr,h->h_length);
  *sock=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  error=connect(*sock, (struct sockaddr*) &temp, sizeof(temp));
  return error;
}



/**
 * \brief Test client that superposes joint torques with sine waves.
 */
class LBRTorqueSineOverlayClient : public LBRClient
{
   
public:
      
   /**
    * \brief Constructor.
    * 
    * @param jointMask Bit mask that encodes the joint indices to be overlaid by sine waves
    * @param freqHz Sine frequency in Hertz
    * @param torqueAmplitude Sine amplitude in Nm
    */
   LBRTorqueSineOverlayClient(unsigned int jointMask, double freqHz, 
         double torqueAmplitude);
   
   /** 
    * \brief Destructor.
    */
   ~LBRTorqueSineOverlayClient();
   
   /**
    * \brief Callback for FRI state changes.
    * 
    * @param oldState
    * @param newState
    */
   virtual void onStateChange(ESessionState oldState, ESessionState newState);
   
   /**
    * \brief Callback for the FRI session state 'Commanding Wait'.
    */
   virtual void waitForCommand();
   
   /**
    * \brief Callback for the FRI state 'Commanding Active'.
    */
   virtual void command();
   

  void get_js(ROBOT_STATE & state);
   void get_trq(ROBOT_STATE & state);
   void set_trq(ROBOT_STATE trq ) {     
      for( int i=0; i<7; i++ )
        _trq.jstate[i] = trq.jstate[i]; 
    }
   
      
private:
      
   int _jointMask;         //!< bit mask encoding of overlay joints
   double _freqHz;         //!< sine frequency (Hertz)
   double _torqueAmpl;     //!< sine amplitude (Nm)
   double _phi;            //!< phase of sine wave
   double _stepWidth;      //!< stepwidth for sine 
   double _torques[LBRState::NUMBER_OF_JOINTS]; //!< commanded superposed torques
   double _jointPosition[LBRState::NUMBER_OF_JOINTS]; //!< commanded superposed torques
   ROBOT_STATE _rs;
   ROBOT_STATE _trq;
   ROBOT_STATE _trq_in;
};

#endif // _KUKA_FRI_LBR_TORQUE_SINE_OVERLAY_CLIENT_H
