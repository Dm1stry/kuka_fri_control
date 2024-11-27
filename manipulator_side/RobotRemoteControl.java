package application;

import java.io.*;
import java.net.*;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.ReentrantLock;


import com.kuka.connectivity.fri.FRIConfiguration;
import com.kuka.connectivity.fri.FRIJointOverlay;
import com.kuka.connectivity.fri.FRISession;
import com.kuka.connectivity.fri.ClientCommandMode;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;


/**
 * Creates a FRI Session.
 */
public class RobotRemoteControl extends RoboticsAPIApplication implements MessageListener
{
    private Controller robotController;
    private LBR robot;
    private String clientName;
    private RemoteConnection connection;
    private Thread connectionThread;
    private Output output;
    
    private ReentrantLock friConfigurationLock;
    private AtomicBoolean friConfigurationInitialized;
    private FRIConfiguration friConfiguration;
    
    private ReentrantLock movementCommandLock;
    private ClientCommandMode commandMode;
    private MoveType moveType;
    private MotionControlMode motionControlMode;
    
    private AtomicBoolean endMotion;
    
    enum MoveType 
    {
    	/* Syncronous movement
    	 * - In each case, the programmed motion stops at the approximation point with exact positioning.
    	 * - The subsequent motion starts from the last commanded position.
    	 * - Command mode is ended after each motion section and reinitialized for the subsequent motion.
    	 * - After end of it fri will shutdown */
    	SYNC,
    	
    	/* Asyncronous movement
    	 * - The motion is continuously executed and approximated.
    	 * - Command mode is not interrupted.
    	 * - After end of it fri will continue running
    	 */
    	ASYNC
    }
    
    enum MotionControlMode 
    {
    	POSITION,  // PTP
    	CARTESIAN_POSITION,  // CartesianPTP
    	JOINT_SPLINE_MOTION,  // Velocity, acceleration and jerk control in cartesian space
    	CARTESIAN_SPLINE_MOTION, // Velocity, acceleration and jerk control in joint space
    	JOINT_IMPEDANCE,  // PosHold in Joint impedance mode
    	CARTESIAN_IMPEDANCE,  // PosHold in cartesian impedance mode
    	CARTESIAN_SINE_IMPEDANCE
    }
    
    
    
    @Override
    public void onMessageReceived(byte[] message, Socket clientSocket)
    {
    	if(message.length < 2)
    	{
    		output.print("Error, message less the 2 bytes");
    	}
    	else if(message[0] == 0x00)
    	{
    		friConfigurationLock.lock();
    		friConfiguration = parseInitializationMessage(message);
    		friConfigurationInitialized.set(true);
    		friConfigurationLock.unlock();
    	}
    	else if(message[0] == 0x01)
    	{
    		movementCommandLock.lock();
    		commandMode = getCommandMode(message);
    		moveType = getMoveType(message);
    		motionControlMode = getMotionControlMode(message);
    		movementCommandLock.unlock();
    	}
    	else if(message[0] == 0x02)
    	{
    		endMotion.set(true);
    	}
    }
    
    private FRIConfiguration parseInitializationMessage(byte[] message)
    {
    	return null;
    }
    
    private ClientCommandMode getCommandMode(byte[] message)
    {
    	return null;
    }
    
    private MoveType getMoveType(byte[] message)
    {
    	return null;
    }
    
    private MotionControlMode getMotionControlMode(byte[] message)
    {
    	return null;
    }
    
    @Override
    public void initialize()
    {
        robotController = (Controller) getContext().getControllers().toArray()[0];
        robot = (LBR) robotController.getDevices().toArray()[0];
        clientName = "172.31.1.150";  // PC address
        output = new KukaOutput(getLogger());
        connection = new RemoteConnection(this, output, 30001);
        connectionThread = new Thread(connection);
    }

    @Override
    public void run()
    {
    	connectionThread.start();
 
    	output.print("Wait for FRI initialization message");
    	while(!(friConfigurationInitialized.get())) {}
    	friConfigurationLock.lock();
        FRISession friSession = new FRISession(friConfiguration);
        friConfigurationLock.unlock();
        output.print("Initialization message arrived");
        output.print("Creating FRI connection to " + friConfiguration.getHostName());
        output.print("SendPeriod: " + friConfiguration.getSendPeriodMilliSec() + "ms |"
              + " ReceiveMultiplier: " + friConfiguration.getReceiveMultiplier());
        output.print("");
       
        while(!(endMotion.get()))
        {
	        output.print("Wait for movement command");
	        movementCommandLock.lock();
	        ClientCommandMode local_commandMode = commandMode;
	        MoveType local_moveType = moveType;;
	        MotionControlMode local_motionControlMode = motionControlMode;
	        movementCommandLock.unlock();
	        FRIJointOverlay jointOverlay = new FRIJointOverlay(friSession, local_commandMode);
	        try
	        {
	            friSession.await(5, TimeUnit.SECONDS);
	        }
	        catch (final TimeoutException e)
	        {
	            getLogger().error(e.getLocalizedMessage());
	            friSession.close();
	            return;
	        }
	        output.print("FRI connection established.");
	        
	
	        // move to start pose
	        robot.move(ptp(Math.toRadians(90), .0, .0, Math.toRadians(90), .0, Math.toRadians(-90), .0));
	
	        // async move with overlay ...
	        robot.moveAsync(ptp(Math.toRadians(-90), .0, .0, Math.toRadians(90), .0, Math.toRadians(-90), .0)
	                .setJointVelocityRel(0.2)
	                .addMotionOverlay(jointOverlay)
	                .setBlendingRel(0.1)
	                );
	
	        // ... blending into sync move with overlay
	        robot.move(ptp(Math.toRadians(90), .0, .0, Math.toRadians(90), .0, Math.toRadians(-90), .0)
	                .setJointVelocityRel(0.2)
	                .addMotionOverlay(jointOverlay)
	                );

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
        final RobotRemoteControl app = new RobotRemoteControl();
        app.runApplication();
    }

}
