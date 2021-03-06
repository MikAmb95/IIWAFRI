package com.kuka.connectivity.fri.example;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import com.kuka.connectivity.fastRobotInterface.ClientCommandMode;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRIJointOverlay;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.CommandInvalidException;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;

/**
 * Moves the LBR in a start position, creates an FRI-Session and executes a
 * PositionHold motion with FRI overlay. During this motion joint angles and
 * joint torques can be additionally commanded via FRI.
 */
public class LBRTorqueSineOverlay extends RoboticsAPIApplication
{
    private Controller _lbrController;
    private LBR _lbr;
    private String _clientName;

    @Override
    public void initialize()
    {
        _lbrController = getController("KUKA_Sunrise_Cabinet_1");;
        _lbr = (LBR) getDevice(_lbrController,
				"LBR_iiwa_14_R820_1");
        // **********************************************************************
        // *** change next line to the FRIClient's IP address                 ***
        // **********************************************************************
        _clientName = "192.170.10.146";
    }

    @Override
    public void run()
    {
        // configure and start FRI session
        FRIConfiguration friConfiguration = FRIConfiguration.createRemoteConfiguration(_lbr, _clientName);
        // for torque mode, there has to be a command value at least all 5ms
        friConfiguration.setSendPeriodMilliSec(3);
        friConfiguration.setReceiveMultiplier(1);

        getLogger().info("Creating FRI connection to " + friConfiguration.getHostName());
        getLogger().info("SendPeriod: " + friConfiguration.getSendPeriodMilliSec() + "ms |"
                + " ReceiveMultiplier: " + friConfiguration.getReceiveMultiplier());

        FRISession friSession = new FRISession(friConfiguration);
        FRIJointOverlay torqueOverlay = new FRIJointOverlay(friSession, ClientCommandMode.TORQUE);

        _lbr.move(ptp(.0, .0, Math.toRadians(90), Math.toRadians(90), .0, Math.toRadians(-90), .0).setJointVelocityRel(0.1));
        // wait until FRI session is ready to switch to command mode
        try
        {
            friSession.await(10, TimeUnit.SECONDS);
        }
        catch (final TimeoutException e)
        {
            getLogger().error(e.getLocalizedMessage());
            friSession.close();
            return;
        }

        getLogger().info("FRI connection established.");

        // move to start pose
       

        // start PositionHold with overlay
        JointImpedanceControlMode ctrMode = new JointImpedanceControlMode(0, 0, 0, 0, 0, 0, 0);
        PositionHold posHold = new PositionHold(ctrMode, -1, TimeUnit.SECONDS);

        
        try{
        	_lbr.move(posHold.addMotionOverlay(torqueOverlay));
        	
        } catch (final CommandInvalidException e) {
        	
        	getLogger().error(e.getLocalizedMessage());
        
        }
       
        

        // done
        friSession.close();
    }

    /**
     * main.
     * 
     * @param args
     *            args
     */
    public static void main(final String[] args)
    {
        final LBRTorqueSineOverlay app = new LBRTorqueSineOverlay();
        app.runApplication();
    }

}