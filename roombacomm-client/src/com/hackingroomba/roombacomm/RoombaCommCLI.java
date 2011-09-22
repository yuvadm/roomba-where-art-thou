/*
 * roombacomm.RoombaCommCLI -- test out RoombaComm library by issuing commands to it from the command line
 *
 *  Copyright (c) 2009 Paul Bouchier, bouchier@at@classicnet.net
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General
 *  Public License along with this library; if not, write to the
 *  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
 *  Boston, MA  02111-1307  USA
 *
 */
package com.hackingroomba.roombacomm;

import java.io.*;
import com.hackingroomba.roombacomm.RobotConnection.ReadTerminator;

public class RoombaCommCLI {
	// robot communications variables
	private String curLine = ""; // Line read from standard in
	private String [] args;
	private String protocol;
	private String portName;
    private RobotConnection robotConnection = null;
    private RobotConnection arduinoConnection = null;
    private RobotConnection localizerConnection = null;
    private RoombaComm robot;		// use ArduinoBot as the robot, because it's the highest class
    private ArduinoBot arduino;    
    private RobotType robotType;
    
	// encoder variables
    int speed = 50;
    int angle;
    int distance = 0;
	int lEncoder = 0;
	int rEncoder = 0;
	int lastLEncoder = 0;
	int lastREncoder = 0;
	int lEncoderOverflow = 0;
	int rEncoderOverflow = 0;
    
	// navigation variables
	private double x = 0;
	private double y = 0; 
	private double targetX, targetY;
	private boolean targetInitialized = false;
	private int localizerCompassOffset = 0;		// angle between localizer Y axis and compass North, add to vectorToStart angle to get compass to target
	private boolean localizerOffsetInitialized = false;
	private double distanceToTarget;
	private int courseToTarget;
	
	//RoboRealm variables
	private watchVideo wv;
	private FrameProcessor fp;
	private int rrShapeX;			// X location in image frame of detected shape from RoboRealm
	private int rrConfidence;
	private int rrSize;
	private int rrRelativeHeading;
	private boolean roborealmConnected = false;

	// I/O variables
	InputStreamReader isr = new InputStreamReader(System.in);
	BufferedReader in = new BufferedReader(isr);
	
	private long startTime;
	private long elapsedTime;
	
	public RoombaCommCLI()
	{
		// do nothing
	}
	// main - it all starts here
	public static void main(String[] args) {
		System.out.println("Enter a command (type 'help' for command listing or 'quit' to exit): ");
		RoombaCommCLI cli = new RoombaCommCLI();
		cli.runCommands();
	}
	
	private void usage() {
		System.out.print(
		"Available Commands\n------------------\n" +
		"help\n" +
		"quit\n" +
		"connect {ip:portnum | serialPort} protocol; protocol is OI or SCI or AR (tankbot) or FR (FrankenRoomba) or MO (Mo'bot)\n" +
		"\tUse AR or MO for Arduino connection - skips roombaInit and does arduinoInit\n" +
		"disconnect Disconnect robot\n" +
		"safe Enter safe mode\n" + 
		"full Enter full mode\n" +
		"speed {defaultSpeed} Set the default speed\n" +
		"spin {angleToSpin} Spins based on timer - YMMV\n" +
		"spinTo (compass heading}\n" +
		"drive {distance in inches} Drives based on timer - YMMV - literally\n" +
		"driveFor {distance in inches} [compass heading] Drives based on wheel encoders and compass\n" +
		"driveToTarget Drive from current position to target\n" +
		"rectangle {length, width} [heading] Drive a rectangle, length is 1st, 3rd leg, width is 2nd, 4th leg, heading is initial direction\n" +
		"squaredance {length, heading} Drive square with side 'length', initial heading 'heading', use localizer to seek to target which should be set before run with setTarget" +
		"squaredance2 {length, heading} Drive square with side 'length', initial heading 'heading'" +
		"stop - Stop robot\n" +
		"compass Print the current compass reading\n" +
		"encoders Print the current wheel encoders reading\n" +
		"currentHeading {heading} Set the offset between compass-reported heading and actual heading\n" +
		"sensors\n" +
		"localizer {IP:Port} Connect to localizer\n" +
		"disconnectLocalizer\n" +
		"localize - get and print localizer fix\n" +
		"setTarget {x y} - save away the target location ((x,y) coordinate in feet) in the target variables\n" +
		"vectorToTarget - compute distance, direction to target\n" +
		"printTarget - Print the current target" +
		"vectorToTarget - compute the angle (in localizer coordinate system) and distance to start point\n" +
		"localizerXBearing {bearing of +X axis} - bearing represents compass bearing of +X axis , i.e. +ve clockwise from north to X axis\n" +
		"roborealm - connect to RoboRealm\n" +
		"rrsnap - snap a frame, send it to RoboRealm, and get SHAPES data from RoboRealm\n" +
		"pointToVisualTarget - rotate to find and point at a visual target\n" +
		"seekToVisualTarget - rotate then travel incrementally to a visual target\n" +
		"test Run tests\n" +
		"Connect notes: Frankenroomba: connect 192.168.11.2 FR; should have sdr.sh and sdar1.sh running on chumby\n" +
		"Mobot: with USB serial cable to Serial3, and arduino on USB1, run sdar1.sh on chumby and connect 192.168.11.2:5002 MO"
		);
	}
		
	private void runCommands()
	{
		while (true){
			// get the command line
			try {
				curLine = in.readLine();				
			} catch (IOException e) {
				// Print out the exception that occurred 
				System.out.println("Error reading line: " + e.getMessage()); 
				System.exit(0);
			}
			
			// split the command line
			args = curLine.split("\\s+");
/*			System.out.println("You typed: " + curLine);
			System.out.println("Args found: " + args.length);
			for (String s: args) {
				System.out.println(s);
			}
*/			
			// parse the command & execute it
			if (args[0].equalsIgnoreCase("quit")) {
				robotConnection.disconnect();
				if (arduinoConnection != null)
					arduinoConnection.disconnect();
				System.exit(0);
			} else if (args[0].equalsIgnoreCase("help")) {
				usage();
/*
 * Connect Commands
 */
			} else if (args[0].equalsIgnoreCase("connect")) {
				connect();
			}  else if (args[0].equalsIgnoreCase("disconnect")) {
				robotConnection.disconnect();
				robot = null;
				if (arduinoConnection != null)
					arduinoConnection.disconnect();
				arduino = null;
/*
 * Roborama commands
 */
			} else if (args[0].equalsIgnoreCase("squareDance")) {
				squareDance();
			} else if (args[0].equalsIgnoreCase("squareDance2")) {
				squareDance2();
			} else if (args[0].equalsIgnoreCase("squareDance3")) {
				squareDance3();
			} else if (args[0].equalsIgnoreCase("tabletrip")) {
					tabletrip();	
			} else if (args[0].equalsIgnoreCase("robocolumbus")) {
				robocolumbus();	
/*
 * Mode Commands
 */
			}   else if (args[0].equalsIgnoreCase("safe")) {
				robot.safe();
			}  else if (args[0].equalsIgnoreCase("full")) {
				robot.full();
			}  else if (args[0].equalsIgnoreCase("speed")) {
				if (args.length != 2) 
					System.out.println("Error: must specify speed");
				else {
					speed = Integer.parseInt(args[1]);
				}
/*
 * Spin Commands
 */
			} else if (args[0].equalsIgnoreCase("spin")) {
				if (args.length != 2)
					System.out.println("Error: must specify angle to spin");
				else {
					// get heading
//					if (protocol.equalsIgnoreCase("FR")) {
//						System.out.print("Heading before turn: ");
//						arduino.printCompass();
//					}
					angle = Integer.parseInt(args[1]);
					int rv = spinByCompass(angle);
					if (rv < 0)
						System.out.println("Error spinning");
					// get heading
					if (protocol.equalsIgnoreCase("FR")) {
						System.out.printf("\nHeading after turn: ");
						arduino.printCompass();
					}

				}
			} else if (args[0].equalsIgnoreCase("spinto")) {
				if (args.length != 2)
					System.out.println("Error: must specify angle to spin");
				else {
					// get heading
					if (protocol.equalsIgnoreCase("FR")) {
						System.out.print("Heading before turn: ");
						arduino.printCompass();
					}
					angle = Integer.parseInt(args[1]);
					spinToHeading(angle);
					// getheading
					if (protocol.equalsIgnoreCase("FR")) {
						System.out.print(" Heading after turn: ");
						arduino.printCompass();
					}

				}
			} else if (args[0].equalsIgnoreCase("randomSpin")) {
				while (true) {
					angle = (int)(Math.random() * 358.0);
					System.out.println("spinning to " + angle);
					spinToHeading(angle);
					try {
						if (System.in.available() != 0){
							robot.stop();
							break;
						}				
					} catch (Exception e) {
						System.out.println("Exception: in System.in.available");
						System.exit(-1);
					} 
				}
/*
 * Drive Commands
 */
			} else if (args[0].equalsIgnoreCase("stop")) {
				System.out.println("Stopping robot");
				robot.stop();
			} else if (args[0].equalsIgnoreCase("drive")) {
				if (args.length == 2) {
					distance = Integer.parseInt(args[1]);
					robot.setSpeed(speed);
					robot.goStraight(distance * 25);	// 25mm per inch, goStraight takes mm
				} else if (args.length == 3) {
					distance = Integer.parseInt(args[1]);
					angle = Integer.parseInt(args[2]);
			        float pausetime = Math.abs((distance * 25) / speed);  // mm/(mm/sec) = sec
			        System.out.println("driving speed " + speed + " angle " + angle + " pausetime " + pausetime);
		        	robot.drive( speed, angle );
			        robot.pause( (int)(pausetime*1000) );
			        robot.stop();
				}
			} else if (args[0].equalsIgnoreCase("drivefor")) {
				if (args.length > 1) {
					distance = Integer.parseInt(args[1]);	
					if (args.length > 2) 
						angle = Integer.parseInt(args[2]);
					else
						angle = -1;	// drive in current direction
					try {
						driveByCompass(distance, angle);						
					} catch (Exception e) {
						robot.stop();
						System.out.println("Error: driveFor took exception, robot stopped");
						e.printStackTrace();
					}
				}
			// drive in a rectangle, by compass
			} else if (args[0].equalsIgnoreCase("rectangle")) {
				rectangle();			
			} else if (args[0].equalsIgnoreCase("driveToTarget")) {
				seekToTarget();
/*
 * Sensor Commands
 */
			}  else if (args[0].equalsIgnoreCase("compass")) {
				arduino.printCompass();
			}  else if (args[0].equalsIgnoreCase("currentHeading")) {
				if (args.length != 2) {
					System.out.println("Error - must specify current heading");
					continue;
				}
				int currentMagHeading = Integer.parseInt(args[1]);
				arduino.setCompassOffset(currentMagHeading);
			} else if (args[0].equalsIgnoreCase("sensors")) {
				if (!robot.updateSensors())
					System.err.println("Error attempting to read valid data from robot.UpdateSensors()");
				else
					System.out.println(robot.getSensorsAsString());
			} else if (args[0].equalsIgnoreCase("initEncoders")) {
				try {
					readEncoderDistance(true);					
				} catch (Exception e) {
					System.err.println("Exception in initEncoders" + e.getMessage());
				}
			} else if (args[0].equalsIgnoreCase("encoders")) {
				double readEncDistance = 0;
				
				startTime = System.currentTimeMillis();
				try {
					readEncDistance = readEncoderDistance(false);
				} catch (Exception e) {
					System.err.println("Exception reading robot encoders: " + e.getMessage());
				}
				long elapsedTime = System.currentTimeMillis() - startTime;
				System.out.println("Distance: " + readEncDistance + " in " + elapsedTime + " ms");

/*
 * Localizer commands
 */
			} else if (args[0].equalsIgnoreCase("localizer")) {
				connectLocalizer();
			} else if (args[0].equalsIgnoreCase("disconnectLocalizer")) {
				localizerConnection.disconnect();
			} else if (args[0].equalsIgnoreCase("localize")) {
				if (!localize()) {
					System.out.println("Error reading localizer");
					continue;
				}
			} else if (args[0].equalsIgnoreCase("setTarget")) {
				if (args.length < 3) {
					System.out.println("Must provide x & y location of target (in feet)");
					continue;
				}
				saveTarget(args[1], args[2]);
			} else if (args[0].equalsIgnoreCase("vectorToTarget")) {
				if (!localize()) {
					System.out.println("Error reading localizer");
					continue;
				}
				try {
					vectorToTarget();					
				} catch (Exception e) {
					System.err.println("Exception computing vector to target - is target initialized?");
					continue;
				}
			} else if (args[0].equalsIgnoreCase("printTarget")) {
				System.out.println("targetX: " + targetX + " targetY: " + targetY);
			} else if (args[0].equalsIgnoreCase("localizerXBearing")) {
				if (args.length != 2) {
					System.out.println("Error - must specify bearing of +X axis");
					continue;
				}
				try {
					// Subtract 90 from localizer X axis (measured by hand-compass)
					localizerCompassOffset = (Integer.parseInt(args[1]) + 270) % 360;
					arduino.writeConfigInt("localizerCompassOffset", localizerCompassOffset);
					System.out.println("Wrote localizerCompassOffset to " + localizerCompassOffset);
					localizerOffsetInitialized = true;
				} catch (Exception e) {
					System.err.println("Error saving localizerCompassOffset\n" + e.getMessage());
				}
/*
 * RoboRealm commands
 */
			} else if (args[0].equalsIgnoreCase("roborealm")) {
				connectRoborealm();
			} else if (args[0].equalsIgnoreCase("rrsnap")) {
				int rv;
				try {
					rv = rrsnap();					
				} catch (Exception e) {
					System.out.println(e.getMessage());
					continue;
				}
				System.out.print("Found shape confidence " + rrConfidence + " at X:" + rrShapeX + " (" 
						+ rrRelativeHeading + " degrees), size " + rrSize);
			} else if (args[0].equalsIgnoreCase("seekToVisualTarget")) {
				seekToVisualTarget(0);
			} else if (args[0].equalsIgnoreCase("pointToVisualTarget")) {
				pointToVisualTarget();
/*
 * Uncategorized commands
 */
			} else if (args[0].equalsIgnoreCase("test")) {
				testHeadingChange();
			} else {
				System.out.println("Invalid command");
			}
		}
	
	}
	/**
	 * Connect to robot using args
	 */
	private void connect() {
		if (args.length > 1) {
			portName = args[1];
		}
		if (args.length > 2) {
			protocol = args[2];
		}
		if ((protocol == null) || (portName == null)) {
			System.out.println("port name and protocol must be specified\n");
			return;
		}

		try {
			if (!connect(portName)) {
				System.out.println("Error connecting to service");
				return;
			}
			if (protocol.equalsIgnoreCase("OI") || protocol.equalsIgnoreCase("SCI") || 
					protocol.equalsIgnoreCase("FR")) {
				robotType = new RobotType(RobotType.robotTypes.roomba);
				initRoomba();						
			} else if (protocol.equalsIgnoreCase("FR")) {	// FrankenRoomba
				robotType = new RobotType(RobotType.robotTypes.frankenRoomba);
				arduino.initArduinoBot(robotType);
			} else if (protocol.equalsIgnoreCase("AR")) {	// Arduino
				robotType = new RobotType(RobotType.robotTypes.tankbot);
				arduino.initArduinoBot(robotType);
			} else if (protocol.equalsIgnoreCase("MO")) {
				robotType = new RobotType(RobotType.robotTypes.mobot);
				arduino.initArduinoBot(robotType);
			} else {
				System.out.println("Invalid protocol");
				return;
			}
		} catch (Exception e) {
			e.printStackTrace();
		}
	}
	/**
	 * Connect to a TCP or serial service, which can have roomba
	 * behind it.
	 * @param portName A string containing a serial port name (e.g. /dev/ttyUSB0, or COM12:) or an IP address
	 * with optional port number after it (e.g. 192.168.1.33:5001).
	 */
	public boolean connect(String portName)
	{
		int preferredPortNum = 5001;
		
		robotConnection = new RobotConnection(portName, preferredPortNum);
		if (!robotConnection.connect()) {
			System.out.println("Couldn't connect to " + portName);
			return false;
		}
		// init robot, which is used for the standard roombacomm commands
		if ((protocol.equalsIgnoreCase("AR")) || (protocol.equalsIgnoreCase("MO"))) {
			robot = new ArduinoBot(robotConnection);	// superclass of roombacomm			
		} else {
			robot = new RoombaCommTCPClient(robotConnection);	// superclass of roombacomm			
		}
		robot.setConnected(true);
		// if FrankenRoomba, set up a 2nd robot and connection
		if (protocol.equalsIgnoreCase("FR")) {
			arduinoConnection = new RobotConnection(portName, 5002);
			if (!arduinoConnection.connect()) {
				System.out.println("Couldn't connect to port 5002 on " + portName);
				return false;
			}
			arduino = new ArduinoBot(arduinoConnection);	// it's actually a superclass of roombacomm
			arduino.setConnected(true);
			System.out.println("Connected to arduino socket");
		} else {
			// this is an uggly hack to cover up that lower level methods use 2 robots.
			// it should be done by creating a new robot type: Frankenroomba, which sends the right commands to
			// the right place
			arduino = (ArduinoBot)robot;
		}
		return true;
	}
	
	/**
	 * Initialize a Roomba
	 */
	public boolean initRoomba()
	{
        System.out.println("Roomba startup");
        robot.startup();
        robot.control();
		robot.pause(1000);

		System.out.println("Checking for Roomba... \n");
		// roombaCommTCPClient.setDebug(true);
		//robot.updateSensors();	// do it once, which doesn't seem to return anything 
		//robot.pause(1000);
		if (robot.updateSensors()) {
			System.out.println("Roomba found!\n");
			System.out.println(robot.getSensorsAsString());
		} else {
			System.out.println("No Roomba. :(  Is it turned on?\n");
			return false;
		}
		// roombacomm.updateSensors();
		System.out.println("connected (" + robot.connected() + ")\n");
		if (robot.connected()) {
			System.out.println("Playing some notes\n");
			robot.playNote(72, 10); // C
			robot.pause(200);
			robot.playNote(79, 10); // G
			robot.pause(200);
			robot.playNote(76, 10); // E
			robot.pause(200);
		}
		robot.setSpeed(speed);

		return true;
	}
	/**
	 * Connect to robot using args
	 */
	private boolean connectLocalizer() {
		String localizerPort = null;
		
		if (args.length > 1) {
			localizerPort = args[1];
		}
		localizerConnection = new RobotConnection(localizerPort, 5010);
		if (!localizerConnection.connect()) {
			System.out.println("Couldn't connect to localizer at: " + portName);
			return false;
		}
		localizerConnection.setReadTimeout(30000);
		System.out.println("Connected to localizer");
		return true;
	}

	/*
	 * Spin an angle
	 * @param angle Angle to spin, positive is clockwise (like a compass)
	 */
	public int spinByCompass(int angle)
	{
		int rv;
		int targetHeading;
		
		targetHeading = arduino.getCompass();
		if (targetHeading < 0)
			return targetHeading;
		targetHeading += angle;
		targetHeading %= 360;
		rv = spinToHeading(targetHeading);
		return rv;
	}
	
	/*
	 * Spin to a heading using the simple algorithm for roomba, tankbot, and PID for Mobot. Mo'bot has
	 * lots of inertia, so needs derivative to prevent overshoot, and operates in high-load environment
	 * like grass, so needs integral to get it moving.
	 * @param heading Desired direction
	 * @return 0 = success
	 */
	public int spinToHeading(int heading)
	{
		if (robotType.robotType == RobotType.robotTypes.mobot) {
			return(spinToHeadingPID(heading));
		} else {
			return (spinToHeadingSimple(heading));
		}
	}
	/*
	 * Spin to a heading
	 * @param target Compass direction to turn to
	 */
	public int spinToHeadingSimple(int target)
	{
		int currentHeading;
		int angleToSpin, absAngleToSpin;
		int spinSpeed;
		
		spinSpeed = robotType.fastSpinSpeed;
		
		if ((target > 359) || (target < 0)) {
			System.out.println("Invalid heading: " + target);
			return -1;
		}
		System.out.print("Heading: ");
		while (true) {
			try{
				currentHeading = arduino.getCompass();
				System.out.print(currentHeading + " ");
				angleToSpin = headingChange(target, currentHeading);
				absAngleToSpin = Math.abs(angleToSpin);
				
				if (currentHeading < 0) {
					System.out.println("Read invalid heading: " + currentHeading);
					robot.stop();
					return -2;			
				}
				
				// spin slow when we get close
				if (absAngleToSpin < 10) {
					spinSpeed = robotType.slowSpinSpeed;
				} else {
					spinSpeed = robotType.fastSpinSpeed;
				}
				
				if ((absAngleToSpin < robotType.tolerance) || (System.in.available() > 0)) {
					robot.stop();
					System.out.println();
					robot.pause(500);	// pause to let motion stop, then recheck heading
					currentHeading = arduino.getCompass();
					angleToSpin = headingChange(target, currentHeading);
					absAngleToSpin = Math.abs(angleToSpin);
					if (absAngleToSpin < robotType.tolerance) {
						return 0;
					}
				}
				if (angleToSpin > 0)
					robot.spinRightAt(spinSpeed);
				else
					robot.spinLeftAt(spinSpeed);
				
				if (System.in.available() != 0){
					robot.stop();
					return -4;
				}
				Thread.sleep(100);	// spin for 100ms
			} catch (Exception e) {
				e.printStackTrace();
				return -3;
			}			
		}
	}

	/*
	 * Spin to a heading with PID control
	 * @param heading Desired direction at end of spin
	 * @return 0 = success, negative for error
	 */
	public int spinToHeadingPID(int target)
	{
		int currentHeading;
		int headingError, absHeadingError;
		int spinSpeed;
		double radiusPidOutput = 0;
		
		if ((target > 359) || (target < 0)) {
			System.out.println("Invalid heading: " + target);
			return -1;
		}

		System.out.println("Spinning to heading " + target + " using PID");
		// initialize the direction control system
		Pid radiusPid = new Pid(robotType.KP, robotType.KI, robotType.KD);

		// spin checking compass every 100ms
		while (true) {
			try{
				startTime = System.currentTimeMillis();

				currentHeading = arduino.getCompass();
				System.out.print(currentHeading + " ");
				headingError = headingChange(target, currentHeading);
				absHeadingError = Math.abs(headingError);
				
				if (currentHeading < 0) {
					System.out.println("Read invalid heading: " + currentHeading);
					robot.stop();
					return -2;			
				}
				
				// stop and re-check if angle to spin < tolerance or user interrupt
				if ((absHeadingError < robotType.tolerance) || (System.in.available() > 0)) {
					robot.stop();
					robot.pause(500);	// pause to let motion stop, then recheck heading
					currentHeading = arduino.getCompass();
					System.out.println(currentHeading);				
					headingError = headingChange(target, currentHeading);
					absHeadingError = Math.abs(headingError);
					if (absHeadingError < robotType.tolerance) {
						return 0;
					}
				}
				
				// compute the PID output given current headingError
				radiusPidOutput = radiusPid.computePid(0, headingError);
				spinSpeed = (int)radiusPidOutput;
				if (spinSpeed > 50) spinSpeed = 50;	// cap spinSpeed
				if (spinSpeed < -50) spinSpeed = -50;	// cap spinSpeed
				System.out.print(" rPidOut " + radiusPidOutput );

				// start spinning
				robot.spin(spinSpeed);
				
				// check for user stop input
				if (System.in.available() != 0){
					robot.stop();
					return -4;
				}
				elapsedTime = System.currentTimeMillis() - startTime;
				System.out.println(" in " + elapsedTime + "ms");

				// spin for 100ms
				// Thread.sleep(100);	
			} catch (Exception e) {
				e.printStackTrace();
				return -3;
			}
		}
	}
	
	public int driveByCompass(double distance) throws Exception
	{
		int rv;
		int direction;
		
		direction = arduino.getCompass();
		if (direction < 0) {
			System.out.println("Read invalid heading: " + direction);
			robot.stop();
			return -3;			
		}
		rv = driveByCompass(distance, direction);
		return rv;
	}
	
	public int driveByCompass(double distance, int direction) throws Exception
	{
		// distance control variables
		double distanceTravelled = 0;
		double startEncDistance = 0;
		double readEncDistance = 0;
		double distanceToGo = 0;
		int heading = 0;
		int headingError = 0;
		// speed control variables: speedRamp is a table of {speed, brakingDistance}
		final int [][] speedRamp = {{20,2},{60,2},{100,2},{140,3},{180,4},{220,9},{260,12},{300,15},{340,19},{380,23},{420,27},{460,33},{500,38}};
		final int maxRamp = 12;	// number of steps in speed ramp
		int rampIndex = 0;	
		boolean rampUp = true;	// start by ramping up
		int currentSpeed;
		// direction control variables
		int [] radiusTable;
		int radiusTableMiddle;	// define the midpoint around which PID swings us
		int radiusTableIndx, radius;
		double radiusPidOutput = 0;
		
		readEncoderDistance(true);		// initialize the encoder reader
		while (direction < 0) {
			System.out.println("getting direction to head in");
			direction = arduino.getCompass();
			if (direction < 0) {
				System.err.println("Error reading compass");
			}			
		}
		System.out.println("Driving " + distance + " inches on heading " + direction);
		// initialize the distance control system
		if (distance <= 0) return -1;
		distanceToGo = distance;		
		readEncDistance = startEncDistance = readEncoderDistance(false);
		if (startEncDistance < 0) {
			System.out.println("FIXME Error reading encoders in driveByCompass()");
			readEncDistance = startEncDistance = readEncoderDistance(false);	// read it a 2nd time
			if (startEncDistance < 0) {
				System.out.println("Second error reading encoders in driveByCompass()");
				return -2;
			}
		}
		currentSpeed = speedRamp[rampIndex][0];
		//System.out.println("Encoders (L, R): " + lEncoder +" " + rEncoder);
		
		// initialize the direction control system
		radiusTable = initRadiusTable();
		radiusTableMiddle = (radiusTable.length/2); // table is always an odd # of elements, point to middle of table
		radiusTableIndx = radiusTableMiddle;		// initial radius is middle of table (straight)
		radius = radiusTable[radiusTableIndx];
		Pid radiusPid = new Pid(robotType.KP, robotType.KI, robotType.KD);
		System.out.printf("Distance: %4.2f to go: %4.2f Speed: %d, ReadEncDist: %4.2f (%d %d) radiusPidOutput: %4.2f Radius %d\n", 
				distanceTravelled, distanceToGo, currentSpeed, readEncDistance, lEncoder, rEncoder, radiusPidOutput, radius);
		
		// turn to point in the right direction
		spinToHeading(direction);	
		robot.pause(500);			
		
		// drive checking sensors every 100ms
		while (true) {
			robot.drive(currentSpeed, radius);
			Thread.sleep(100);	// drive for 100ms

			// get encoders & compute distance remaining
			readEncDistance = readEncoderDistance(false);
			if (readEncDistance < 0) {
				System.err.println("FIXME Error reading encoders in driveByCompass()");
				readEncDistance = readEncoderDistance(false);	// read it a 2nd time
				if (startEncDistance < 0) {
					System.out.println("Second error reading encoders in driveByCompass()");
					continue;
				}				
			}
			distanceTravelled = readEncDistance - startEncDistance;
			distanceToGo = distance - distanceTravelled;
			heading = arduino.getCompass();
			if (heading < 0) {
				System.err.println("Error reading compass");
				continue;
			}
			//System.out.println("Encoders (L, R, distance): " + lEncoder +" " + rEncoder + " " + currentDistance);

			// ramp speed up then down
			if (distanceToGo < speedRamp[rampIndex][1]) {	// if closer than braking distance, decelerate to Vmin
				rampUp = false;
				if (rampIndex > 0)
					rampIndex--;
			} else if (distanceToGo > speedRamp[rampIndex][1]) { 	// if further away than braking distance, accelerate to Vmax
				if ((rampIndex < maxRamp) && (rampUp == true))
					rampIndex++;
			}
			currentSpeed = speedRamp[rampIndex][0];

			heading = arduino.getCompass();
			headingError = headingChange(direction, heading);
			radiusPidOutput = radiusPid.computePid(0, headingError);
			radiusTableIndx = (int)radiusPidOutput + radiusTableMiddle;		// offset PID output into table
			if (radiusTableIndx < 0) radiusTableIndx = 0;
			else if (radiusTableIndx >= radiusTable.length) radiusTableIndx = radiusTable.length -1;
			radius = radiusTable[radiusTableIndx];

			//System.out.printf("Distance: %4.2f to go: %4.2f Heading: %d Speed: %d, ReadEncDist: %4.2f (%d %d) radiusPidOutput: %4.2f Radius %d\n", 
			//		distanceTravelled, distanceToGo, heading, currentSpeed, readEncDistance, lEncoder, rEncoder, radiusPidOutput, radius);

			// stop if we've arrived
			if ((distance < distanceTravelled) || (System.in.available() > 0) || (readEncDistance < 0)) {
				robot.stop();
				System.out.println("Finished trip, travelled: " + distanceTravelled + " in, readEncDistance: " + readEncDistance);
				robot.pause(500);	// pause to let motion stop, then recheck distance
				break;
			}
		}
		return 0;
	}

	/**
	 * Drive to a target that has been previously set using the settarget command.
	 * Use the localizer to decide how far & which direction to drive
	 */
	private void seekToTarget() {
		int currentHeading;
		while ((currentHeading = arduino.getCompass()) < 0) 
			;
		seekToTarget(currentHeading, 0);

	}
	private void seekToTarget(int startHeading, long timeLimit) {
		// measure/compute initial distance & direction to target
		try {
			while (!localize())
				robot.goForward(12 * 25);	// go forward a foot
			vectorToTarget();		// compute the distance and course to target				
		} catch (Exception e) {
			System.err.println("Exception getting vector to target\n" + e.getMessage());
			return;
		}

		// seek to target while further away than tolerance, limited by timeLimit if non-zero
		while ((distanceToTarget > 2.0) && ((timeLimit == 0) || (timeLimit > System.currentTimeMillis()))) {
			int rv = spinToHeading(courseToTarget);
			if (rv < 0) {
				System.out.println("Error spinning to target: " + rv);
				return;					
			}
			try {
				rv = driveByCompass(distanceToTarget, courseToTarget);					
				if (rv < 0) {
					System.out.println("Error driving to target: " + rv);
				}
			} catch (Exception e) {
				System.out.println("Error driving to target: " + e.getMessage()); 
				e.printStackTrace();
				return;
			}			
			// measure/compute current distance & direction to target
			try {
				while (!localize())
					robot.goForward(12 * 25);	// go forward a foot
				vectorToTarget();		// compute the distance and course to target				
			} catch (Exception e) {
				System.err.println("Exception getting vector to target\n" + e.getMessage());
				return;
			}
		}
		spinToHeading(startHeading);
		if (timeLimit > System.currentTimeMillis()) {
			System.out.println("**** Timed out while seeking target ****");
		} else {
			System.out.println("**** Arrived at target ****");			
		}
		return;
	}

	/**
	 * Drive in a rectangle specified by length (1st & 3rd legs) & width (2nd & 4th legs) with optional heading
	 */
	private void rectangle() {
		int length, width;

		if (args.length < 3) {
			System.out.println("Error: must specify length, width");
			return;
		}
		length = Integer.parseInt(args[1]);
		width = Integer.parseInt(args[2]);
		if (args.length == 4)
			angle = Integer.parseInt(args[3]);
		else
			angle = arduino.getCompass();	// if initial heading not specified, use current heading
		if (angle < 0) {
			System.out.println("initial heading must be > 0, was: " + angle);
			return;
		}
		System.out.println("Driving rectangle length: " + length + " width: " + width + "initial heading: " + angle);
		try {
			for (int i=0;i<2;i++) {
				driveByCompass(length, angle);
				angle += 90;
				angle %= 360;
				driveByCompass(width, angle);
				angle += 90;
				angle %= 360;						
			}
			spinToHeading(angle);
		} catch (Exception e) {
			System.out.println("Exception driving");
			e.printStackTrace();
		}
		return;
	}

	/**
	 * Drive in a square specified by length with optional heading
	 */
	private void squareDance() {
		int length;
		int startHeading;

		if (args.length < 2) {
			System.out.println("Error: must specify length of side");
			return;
		}
		length = Integer.parseInt(args[1]);
		startHeading = arduino.getCompass();	// record what direction to turn to at finish
		
		if (args.length == 3)
			startHeading = angle = Integer.parseInt(args[2]);
		else
			angle = startHeading;	// if initial heading not specified, use current heading
		if (angle < 0) {
			System.out.println("initial heading must be > 0, was: " + angle);
			return;
		}
		System.out.println("Driving square dance length: " + length + "initial heading: " + angle);
		startTime = System.currentTimeMillis();
		long timeLimit = System.currentTimeMillis() + (2500 * 60);	// limit square dance to 2.5 minutes to avoid running overtime
		
		try {
			// drive 4 sides
			for (int i=0;i<4;i++) {
				driveByCompass(length, angle);
				angle += 90;
				angle %= 360;
			}
			
			seekToTarget(startHeading, timeLimit);
			
		} catch (Exception e) {
			System.out.println("Exception driving");
			e.printStackTrace();
		}
		spinToHeading(startHeading);
		return;
	}
	
	/*
	 * Drive squaredance just based on time, without seek to target at end. 
	 * Don't use compass to control direction, no timeLimit
	 */
	private void squareDance2() {
		int length;
		int startHeading;

		if (args.length < 2) {
			System.out.println("Error: must specify length of side");
			return;
		}
		length = Integer.parseInt(args[1]);
		startHeading = arduino.getCompass();	// record what direction to turn to at finish
		
		if (args.length == 3)
			startHeading = angle = Integer.parseInt(args[2]);
		else
			angle = startHeading;	// if initial heading not specified, use current heading
		if (angle < 0) {
			System.out.println("initial heading must be > 0, was: " + angle);
			return;
		}
		System.out.println("Driving square dance length: " + length + "initial heading: " + angle);
		startTime = System.currentTimeMillis();
		
		try {
			// drive 4 sides
			robot.speed =200;
			for (int i=0;i<4;i++) {
				robot.goStraight(length * 25);
				angle += 90;
				angle %= 360;
				spinToHeading(angle);
			}						
		} catch (Exception e) {
			System.out.println("Exception driving");
			e.printStackTrace();
		}
		spinToHeading(startHeading);
		return;

	}
	/*
	 * Drive squaredance just based on time, without seek to target at end. 
	 * Don't use compass to control direction, no timeLimit
	 */
	private void squareDance3() {
		int length;
		int startHeading;

		if (args.length < 2) {
			System.out.println("Error: must specify length of side");
			return;
		}
		length = Integer.parseInt(args[1]);
		startHeading = arduino.getCompass();	// record what direction to turn to at finish
		
		if (args.length == 3)
			startHeading = angle = Integer.parseInt(args[2]);
		else
			angle = startHeading;	// if initial heading not specified, use current heading
		if (angle < 0) {
			System.out.println("initial heading must be > 0, was: " + angle);
			return;
		}
		System.out.println("Driving square dance length: " + length + "initial heading: " + angle);
		startTime = System.currentTimeMillis();
		long timeLimit = System.currentTimeMillis() + (2500 * 60);	// limit square dance to 2.5 minutes to avoid running overtime
		
		try {
			// drive 4 sides
			robot.speed =200;
			for (int i=0;i<4;i++) {
				robot.goStraight(length * 25);
				angle = arduino.getCompass();
				angle += 90;
				angle %= 360;
				spinToHeading(angle);
			}
			
		} catch (Exception e) {
			System.out.println("Exception driving");
			e.printStackTrace();
		}
		seekToTarget(startHeading, timeLimit);

		
		spinToHeading(startHeading);
		return;

	}

	/**
	 * Drive to edge of table, back up, turn around, & do it to the other end, but don't fall off
	 */
	private void tabletrip() 
	{
		robot.goForwardAt(100);
		robot.pause(15000);
		robot.full();
		robot.pause(500);
		robot.goBackward(50);
		robot.safe();
		robot.pause(500);
		System.out.println("safe, spinning");
		spinByCompass(180);
		robot.goForwardAt(100);
		robot.pause(15000);
		robot.full();
		robot.pause(500);
		robot.goBackward(50);
		robot.safe();
		System.out.println("safe, spinning");
		robot.pause(500);
		spinByCompass(180);
	}
	
	/**
	 * RoboColumbus: drive a distance, turn on video & snap pic, find target in image using roborealm, & incrementally seek
	 * to target, re-acquiring image every so often
	 */
	private void robocolumbus() 
	{
		int initialDriveDistance = 12;
		int startHeading;
		boolean rv;
		
		// set up parameters
		if (args.length < 1) {
			System.out.println("Error: must specify drive distance (feet) before seeking target");
			return;
		}
		//initialDriveDistance = Integer.parseInt(args[1]) * 12;
		startHeading = arduino.getCompass();	// record what direction to face when searching for target
		
		angle = 83;
		//angle = ;
		initialDriveDistance = 95;
		if (args.length == 2) {
			initialDriveDistance = 12 * Integer.parseInt(args[1]);
		}
		if (args.length == 3)
			startHeading = angle = Integer.parseInt(args[2]);
		else
			angle = startHeading;	// if initial heading not specified, use current heading
		if (angle < 0) {
			System.out.println("initial heading must be > 0, was: " + angle);
			return;
		}
		
		
		System.out.println("Driving RoboColumbus length: " + initialDriveDistance + "initial heading: " + angle);
		startTime = System.currentTimeMillis();
		long timeLimit = System.currentTimeMillis() + (4500 * 60);	// limit square dance to 2.5 minutes to avoid running overtime
		
		// start driving
		try {
			// drive to image search point
			spinToHeading(angle);
			robot.speed = 200;
			driveByCompass(initialDriveDistance*12, angle);
			
			// point to the start direction for the image search
			spinToHeading(angle);
			
			rv = seekToVisualTarget(timeLimit);
			if (!rv) {
				System.out.println("FAILED: unable to visually acquire target - stopping");
				System.exit(-1);
			}
		} catch (Exception e) {
			System.out.println("Exception driving");
			e.printStackTrace();
		}
		return;
		
	}

	private boolean seekToVisualTarget(long timeLimit) 
	{
		double incrementalDistance = 12.0;
		boolean arrived = false;
		int currentSeekHeading;
		
		currentSeekHeading = arduino.getCompass();	// starting direction for looking for targe
		// measure/compute initial distance & direction to target

		while (!arrived) {
			currentSeekHeading = pointToVisualTarget();	// update current heading to target
			try {
				if (System.in.available() != 0){
					robot.stop();
					return false;
				}				
			} catch (Exception e) {
				System.out.println("Exception: in System.in.available");
				System.exit(-1);
			}
			
			int rv = spinToHeading(currentSeekHeading);
			if (rv < 0) {
				System.out.println("Error spinning to target: " + rv);
				return false;					
			}
			try {
				rv = driveByCompass(incrementalDistance, currentSeekHeading);					
				if (rv < 0) {
					System.out.println("Error driving to target: " + rv);
				}
			} catch (Exception e) {
				System.out.println("Error driving to target: " + e.getMessage()); 
				e.printStackTrace();
				return false;
			}			
		}
		return true;
	}
	
	/*
	 * Point robot at visual target. On exit, provide compass heading to target, and robot is pointed
	 * approximately at target
	 * @param currentHeading The starting direction we're approximately facing in, and the direction
	 * around which the algorithm searches for the target
	 */
	public int pointToVisualTarget()
	{
		int targetHeading;
		int currentSeekHeading, startHeading;
		int currentSeekHeadingIncrement = 5;
		int seekIteration;
		final int seekLimit = 24;	// how many seeks until we give up
		boolean gotDataFlag;
		
		currentSeekHeading = arduino.getCompass();
		startHeading = currentSeekHeading;
		
		// seek back and forth until we acquire image with good confidence
		for (seekIteration=1; seekIteration<seekLimit; seekIteration++) {
			try {
				if (System.in.available() != 0){
					robot.stop();
					System.out.println("Exception: aborted by user keystroke");
					System.exit(-1);
				}				
			} catch (Exception e) {
				System.out.println("Exception: in System.in.available");
				System.exit(-1);
			}
			try {
				rrsnap();	
				gotDataFlag = true;
			} catch (Exception e) {
				System.out.print(e.getMessage());
				gotDataFlag = false;
			} 
			if ((gotDataFlag == false) || (rrConfidence < 50)) {
				System.out.println("; failed to find target on iteration " + seekIteration + "in direction " + currentSeekHeading);
				//currentSeekHeadingIncrement = 0 - (seekIteration * currentSeekHeadingIncrement); // next direction to look in
				if (seekIteration == 12) {
					currentSeekHeading = startHeading;	//Go back to initial direction & search the other way
					currentSeekHeadingIncrement = 0 - currentSeekHeadingIncrement;
				}
				currentSeekHeading += currentSeekHeadingIncrement;	// failed to find target
				currentSeekHeading = normalizeCompassHeading(currentSeekHeading);	// fixup wraparound
				spinToHeading(currentSeekHeading);		// point in the next direction to search
			} else {
				System.out.println("Found target with robot pointed in direction " + currentSeekHeading);
				break;
			}
		}
		
		if (seekIteration == seekLimit) {
			System.out.println("Couldn't find target: abandoning search");
			return -1;
		}
		
		// return the target heading
		targetHeading = arduino.getCompass() + rrRelativeHeading;
		targetHeading = normalizeCompassHeading(targetHeading);
		System.out.println("pointToVisualTarget returning target at " + targetHeading + " robot direction " + arduino.getCompass());
		return targetHeading;
	}
	
	private int normalizeCompassHeading(int heading)
	{
		if (heading > 359)
			return (heading % 360);
		else if (heading < 0)
			return (heading + 360);
		return heading;
	}
	
	/*
	 * Ask the localizer for robot location. Results returned in x, y class variables
	 * @return boolean which is false if localization failed, true otherwise
	 */
	private boolean localize()
	{
		String locationString = "";
		
		
		if (localizerConnection == null) { 
			// if no localizer, prompt for a location
			System.out.println("Enter location in feet (decimals ok), x and y, in form like x 1.2 2.9<enter>");
			try {
				locationString = in.readLine();				
			} catch (IOException e) {
				// Print out the exception that occurred 
				System.out.println("Error reading line: " + e.getMessage()); 
				return false;
			}
		} else {
			// we are connected to localizer, ask it for location
			localizerConnection.send('L');	// send localize command to localizer
	    	try {
	        	locationString = localizerConnection.readBotToTerminator(ReadTerminator.NULL);   		
	    	} catch (Exception e) {
	    		System.out.println("Error: exception reading localizer");
	    		return false;
	    	}			
		}
    	
    	// split the string apart & look for the Invalid keyword
    	System.out.println("received string: " + locationString);
    	String [] splitLocationStrings = locationString.split("\\s");
    	if (splitLocationStrings[0].compareTo("bogus") == 0) {
    		System.out.println("Location is invalid");
    		return false;
    	}
    	
    	// parse the location string into x & y and print
    	if (splitLocationStrings.length < 3) {
    		System.out.println("Error: must have at least 3 elements in location string");
    		return false;
    	}
    	try {
    		x = new Double(splitLocationStrings[1]) * 12;
    		y = new Double(splitLocationStrings[2]) * 12;    		
    	} catch (Exception e) {
    		System.err.println("Error parsing location strings\n" + e.getMessage());
    		return false;
    	}
		System.out.println("x: " + x + " y: " + y);
		return true;
	}
	private void saveTarget(String x, String y)
	{
		// target is entered in decimal feet, usually from localizer reading
		targetX = Double.parseDouble(x) * 12;
		targetY = Double.parseDouble(y) * 12;
		try {
			arduino.writeConfigDouble(new String("targetX"), targetX);			
			arduino.writeConfigDouble(new String("targetY"), targetY);	
			targetInitialized = true;
		} catch (Exception e) {
			System.err.println("Error writing target location");
		}
	}
	public void vectorToTarget () throws Exception
	{
		double deltaX, deltaY;
		double localizerDirection = 0;
		int angle2t;

		// if we haven't initialized the target location yet, try to read from a config file
		// targetInitialized ensures we only try this once
		try {
			if (!targetInitialized) {	// get the compass offset from file if it exists
				targetX = arduino.readConfigDouble(new String("targetX"));
				targetY = arduino.readConfigDouble(new String("targetY"));
				System.out.println("target set by file read to x: " + targetX + " y: " + targetY);
				targetInitialized = true;
			}
			if (!localizerOffsetInitialized) {	// get the localizer Y axis offset from North if it exists
				localizerCompassOffset = arduino.readConfigInt("localizerCompassOffset");
				localizerOffsetInitialized = true;
				System.out.println("localizerCompassOffset set by file to " + localizerCompassOffset);
			}
		} catch (Exception e) {
			System.out.println("No targetX or targetY file or localizer compass offset file could be opened\n" + e.getMessage());
			throw (e);
		}
		
		// compute distance from current location to target
		deltaX = targetX - x;
		deltaY = targetY - y;
		distanceToTarget = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
		if (deltaX == 0) {		// make sure we don't try to divide by zero
			if (deltaY > 0)
				localizerDirection = Math.PI/2;
			else
				localizerDirection = -Math.PI/2;
		} else {	// deltaX is non-zero, compute arctan to find angle to target
			localizerDirection = Math.atan(deltaY/deltaX);	// in radians, with incrementing positive values going counterclockwise
			if ((deltaX < 0) && (deltaY <=0)) localizerDirection -= Math.PI;
			if ((deltaX < 0) && (deltaY > 0)) localizerDirection += Math.PI;			
		}
		angle2t = (int)Math.toDegrees(localizerDirection);	// angle to start in localizer frame of reference, +180 to -180
		courseToTarget = 90 - angle2t + localizerCompassOffset;	// convert to +ve Y axis = 0 degrees, increasing angle clockwise
		courseToTarget %= 360;			// make sure course < -360
		if (courseToTarget < 0) courseToTarget += 360;	// turn -90 into 270
		System.out.println("** Current location: x: " + x + " y: " + y + " Target location x: " + targetX + " y: " + targetY);
		System.out.println("** Distance to target: " + distanceToTarget + " heading: " + courseToTarget);
		System.out.println("localizerCompassOffset: " + localizerCompassOffset);
	}
	
	private int [] initRadiusTable()
	{
		// speed table goes in steps of 100 from -500 to -2000, with 0x8000 instead of 0
		// then 2000 to 500
		int stepSize = 100;
		int minRadius = 500;
		int maxRadius = 2000;
		int numEntries = (((maxRadius-minRadius) * 2)/stepSize) + 3;	// +1 for straight, +1 on each side for inclusive entries
		int [] radiusTable = new int[numEntries];
		int indx = 0;
		int tableVal = -500;
		
		do {
			radiusTable[indx++] = tableVal;
			tableVal -= stepSize;	// -500, -600 ... -2000
		} while (tableVal >= -maxRadius);
		tableVal = 0x8000;			// mid-value is 0x8000
		radiusTable[indx++] = tableVal;
		tableVal = maxRadius;
		do {
			radiusTable[indx++] = tableVal;
			tableVal -= stepSize;	// 2000, 1900, ... 500
		} while (tableVal >= 500);
		
		return radiusTable;
	}
	/**
	 * 
	 * @param init If true, init initializes encoder variables, otherwise regular read encoder distance
	 * @return Average of encoders distance, in inches
	 * @throws Exception
	 */
	public double readEncoderDistance(boolean init) throws Exception
	{
		byte [] encoders = {43, 44};
		byte [] sensor_bytes;
		double encoderDistance;
		
		if ((protocol.equalsIgnoreCase("AR")) || (protocol.equalsIgnoreCase("MO"))) {
			// FIXME need to handle overflow
			encoderDistance = arduino.getEncoders();
			lEncoder = rEncoder = arduino.getEncoder();
		} else {
			// 	WARNING - THIS DOESN'T SEEM TO HANDLE OVERFLOW WELL - WON'T WORK WITH TANKBOT
			// read encoders from roomba
			robot.queryList(encoders, 4);
			if (robot.getSensorData(4) == false)	// read failed
				return -1;
			sensor_bytes = robot.getSensor_bytes();
			
			// handle overflow in encoders
			lastLEncoder = lEncoder;		// first time through will be 0, then will be last reading;
			lEncoder = ArduinoBot.toUnsignedShort(sensor_bytes[0], sensor_bytes[1]);
			lEncoder += lEncoderOverflow;		
			lastREncoder = rEncoder;
			rEncoder = ArduinoBot.toUnsignedShort(sensor_bytes[2], sensor_bytes[3]);
			rEncoder += rEncoderOverflow;
			
			if (init) {
				lEncoderOverflow = 0;
				rEncoderOverflow = 0;			
				lEncoder = ArduinoBot.toUnsignedShort(sensor_bytes[0], sensor_bytes[1]);
				rEncoder = ArduinoBot.toUnsignedShort(sensor_bytes[2], sensor_bytes[3]);
				lastLEncoder = lEncoder;	// initialize last value to current reading
				lastREncoder = rEncoder;
			} else {
				if ((lEncoder - lastLEncoder) < -1000) {
					lEncoderOverflow += 1<<16;	// add 65536 to overflow adjuster if encoder rolled over
					lEncoder += 1<<16;
				}
				if ((rEncoder - lastREncoder) < -1000) {
					rEncoderOverflow += 1<<16;	// add 65536 to overflow adjuster if encoder rolled over
					rEncoder += 1<<16;
				}
			}
			// average the encoders & turn into inches
		}

		encoderDistance = ((lEncoder + rEncoder)/2) /robotType.countsPerInch;			
		//System.out.println("encoderDistance: " + encoderDistance + " lEncoder: " + lEncoder + " rEncoder: " + rEncoder + " lastLEncoder: " + lastLEncoder + " lEncoderOverflow: " + lEncoderOverflow
		//		+ " lastREncoder: " + lastREncoder + " rEncoderOverflow: " + rEncoderOverflow);
		return encoderDistance;
	}
	
	/**
	 * Calculate the shortest heading change which will move the bot from current heading to target.
	 * See http://www.dreamincode.net/forums/topic/163469-calculating-the-difference-between-two-angles/
	 * for some concepts.
	 * 
	 * @param target Desired compass heading: 0 - 359 degrees
	 * @param heading Current compass heading: 0 - 359 degrees
	 * @return direction and angle to turn. Positive is clockwise (like a compass)
	 */
	public int headingChange(int target, int heading)
	{
		int diff = target - heading;
		if (diff > 180) diff -= 360;
		else if (diff < -180) diff += 360;
		return diff;
	}
	private void testHeadingChange()
	{
		int heading[] = {35, 45, 315, 45, 0, 180, 45};
		int target[] = {45, 35, 45, 315, 180, 0, 45};
		int diff;
		int testCaseCount = 7;
		for (int i=0; i<testCaseCount; i++) {
			diff = headingChange(target[i], heading[i]);
			System.out.println("Target: " + target[i] + " Heading: " + heading[i] + " Heading Change: " + diff);
		}
		
		// first test of vectorToStart: x,y = 0,0, ring then sit on top of x,y
		int xTest[] = {0, 10, 10,  10,   0, -10, -10, -10, 0};
		int yTest[] = {10, 10, 0, -10, -10, -10,   0,  10, 0};
		x = 0; y = 0;
		saveTarget("0", "0");
		for (int i=0; i<xTest.length; i++) {
			x = xTest[i]; y = yTest[i];
			try {
				vectorToTarget();				
			} catch (Exception e) {
				System.err.println("Exception in vectorToTarget");
				System.exit(0);
			}
			System.out.println("x: " + x + " y: " + y + " distanceToTarget: " + distanceToTarget + " courseToTarget: " +
					courseToTarget);
			
		}
	}
	/**
	 * Initialize connection to Roborealm
	 */
	private void connectRoborealm() {
		wv = new watchVideo();
		wv.setHeight(240);
		wv.setWidth(320);
		String s[] = portName.split(":");	// in case portnumber was specified, get the IP by itself
		wv.setVideoServer(s[0]);
		wv.setColor(true);
		wv.setRoborealm(true);
		fp = wv.initVideo();
		System.out.println("Connected to video source and RoboRealm");
		roborealmConnected = true;
	}

	/**
	 * Snap a webcam image and send it to roborealm for analysis and print the results
	 */
	public int rrsnap() throws Exception {
		double conf;
		
		if (!roborealmConnected)
			connectRoborealm();
		
		wv.getShowFrame();
		Boolean rv = fp.frame2Roborealm();	
		if (rv == false) {
			System.out.println("error sending image to Roborealm");
			throw new Exception("Error sending image to Roborealm");
		}
		String rrString = fp.getShapeData();
		if (rrString != null) {
			String s[] = rrString.split(",");  
			int xL, xR;
			xL = Integer.parseInt(s[3]);
			xR = Integer.parseInt(s[4]);
			rrShapeX = (xL + xR)/2;
			rrRelativeHeading = (rrShapeX - (wv.getWidth()/2))/16;	// distance from image center
			conf = Double.parseDouble("0." + s[0]);
			conf *= 100;		// convert to %
			rrConfidence = (int)conf;
			rrSize = Integer.parseInt(s[2]);
			
			//System.out.print("Found shape confidence " + s[0] + " at " + rrShapeX);
			return rrRelativeHeading;
		} else {
			//System.out.println("No RobotRealm data returned");
			throw new Exception("Error: no Roborealm data returned");
		}
	}

}
