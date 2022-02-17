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
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "LBRTorqueSineOverlayClient.h"


using namespace std;

#ifndef M_PI
#define M_PI 3.14159265358979
#endif

//******************************************************************************
LBRTorqueSineOverlayClient::LBRTorqueSineOverlayClient(unsigned int jointMask, 
      double freqHz, double torqueAmplitude) 
   :_jointMask(jointMask)
   , _freqHz(freqHz)
   , _torqueAmpl(torqueAmplitude)
   , _phi(0.0)
   , _stepWidth(0.0)
{
   printf("LBRTorqueSineOverlayClient initialized:\n"
         "\tjoint mask: 0x%x\n"
         "\tfrequency (Hz): %f\n"
         "\tamplitude (Nm): %f\n",
         jointMask, freqHz, torqueAmplitude);
   //cout<<"here"<<endl;
   for(int i = 0; i< LBRState::NUMBER_OF_JOINTS; i++){ _torques[i] = 0.0;}
}

//******************************************************************************
LBRTorqueSineOverlayClient::~LBRTorqueSineOverlayClient()
{
}
      
//******************************************************************************
void LBRTorqueSineOverlayClient::onStateChange(ESessionState oldState, ESessionState newState)
{
   LBRClient::onStateChange(oldState, newState);
   // (re)initialize sine parameters when entering Monitoring
   switch (newState)
   {
      case MONITORING_READY:
      {
         for(int i = 0; i< LBRState::NUMBER_OF_JOINTS; i++){ _torques[i] = 0.0;}
         _phi = 0.0;
         _stepWidth = 2 * M_PI * _freqHz * robotState().getSampleTime();
         break;
      }
      default:
      {
         break;
      }
   }
}
//******************************************************************************

void LBRTorqueSineOverlayClient::waitForCommand()
{
   // In waitForCommand(), the joint values have to be mirrored. Which is done, by calling
   // the base method.
   LBRClient::waitForCommand();
   cout<<"WAIT"<<endl;
   // If we want to command torques, we have to command them all the time; even in
   // waitForCommand(). This has to be done due to consistency checks. In this state it is 
   // only necessary, that some torque values are sent. The LBR does not take the 
   // specific value into account.
   if (robotState().getClientCommandMode() == TORQUE)
   {
      robotCommand().setTorque(_torques);
   }
}
//******************************************************************************

void LBRTorqueSineOverlayClient::get_js(ROBOT_STATE & state) {
   memcpy(_rs.jstate, robotState().getMeasuredJointPosition(), 7 * sizeof(double));
   state = _rs;
}


void LBRTorqueSineOverlayClient::get_trq(ROBOT_STATE & state) {
  memcpy(_trq_in.jstate, robotState().getCommandedTorque(), 7 * sizeof(double));
  state = _trq_in;
}



void LBRTorqueSineOverlayClient::command()
{
    // In command(), the joint values have to be sent. Which is done by calling
    // the base method.
    LBRClient::command();
    
    // Check for correct ClientCommandMode.
    if (robotState().getClientCommandMode() == TORQUE)
    { 
       // calculate  offset
       
        double trq_cmd[7];
        
        cout<<"TRQ_C: ";
        for(int i = 0;i<7;i++){
        trq_cmd[i] = _trq.jstate[i];
        cout<<trq_cmd[i]<<" ";
        }
        cout<<endl;
          

       for (int i=0; i<LBRState::NUMBER_OF_JOINTS; i++)
       {
          if (i == 0)
          {
              _torques[i] = trq_cmd[i];
              //cout<<"TRQ1: "<<trq_cmd[i]<<endl;
              
          }

           if (i == 1)
          {
              _torques[i] = trq_cmd[1];
              //cout<<"TRQ3: "<<trq_cmd[i]<<endl;
              
          }

          if (i == 2)
          {
              _torques[i] = trq_cmd[i];
              //cout<<"TRQ3: "<<trq_cmd[i]<<endl;
              
          }

          if (i == 3)
          {
              _torques[i] = trq_cmd[i];
              //cout<<"TRQ4: "<<trq_cmd[i]<<endl;
              
          }

          if (i == 4)
          {
              _torques[i] = trq_cmd[i];
              //cout<<"TRQ5: "<<trq_cmd[i]<<endl;
              
          }
  
          if (i == 5)
          {
              _torques[i] = trq_cmd[i];
              //cout<<"TRQ6: "<<trq_cmd[i]<<endl;
              
          }

          if (i == 6)
          {
              _torques[i] = trq_cmd[i];
              //cout<<"TRQ7: "<<trq_cmd[i]<<endl;
              
          }
       }
       for (int i=0; i<LBRState::NUMBER_OF_JOINTS; i++)
       {
       _jointPosition[i] = robotState().getMeasuredJointPosition()[i];
        }
       // Set superposed joint torques.
       robotCommand().setTorque(_torques);
       robotCommand().setJointPosition(_jointPosition);
    }
}
