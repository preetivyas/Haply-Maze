/**
 **********************************************************************************************************************
 * @file       sketch_4_Wall_Physics.pde
 * @author     Steve Ding, Colin Gallacher
 * @version    V4.1.0
 * @date       08-January-2021
 * @brief      wall haptic example using 2D physics engine 
 
 sketch_4_Wall_Physics.pde is a new version of this file edited by Preeti Vyas
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */



/* library imports *****************************************************************************************************/ 
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
/* end library imports *************************************************************************************************/  



/* scheduler definition ************************************************************************************************/ 
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/ 



/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 5;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           renderingForce                      = false;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 



/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = 40.0;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                = new PVector(0, 0); 

/* World boundaries in centimeters */
FWorld            world;
float             worldWidth                          = 30.0;  
float             worldHeight                         = 15.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;

float             gravityAcceleration                 = 980; //cm/s2

/* Initialization of virtual tool */
HVirtualCoupling  s;
PImage            haplyAvatar;

/* Initialization of wall */
FBox              w1, w2, w3, w4, w5, w6, w7, w8;
FBox              r1, r2, r3, r4, finish;
FBox              l1;

/* define start and stop button */
FCircle           c1;
FCircle           c2;

/* define game start */
boolean           gameStart                           = false;

/* text font */
PFont             f;



/* end elements definition *********************************************************************************************/ 



/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(1200, 600);
  
  
  /* set font type and size */
  f                   = createFont("Arial", 16, true);
  
  /* device setup */
  
  /**  
   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
   * to explicitly state the serial port will look like the following for different OS:
   *
   *      windows:      haplyBoard = new Board(this, "COM10", 0);
   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
   */ 
  haplyBoard          = new Board(this, "COM4", 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();
  
  widgetOne.set_mechanism(pantograph);
  
  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);
 
  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);
  
  
  widgetOne.device_set_parameters();
  
  
  /* 2D physics scaling and world creation */
  hAPI_Fisica.init(this); 
  hAPI_Fisica.setScale(pixelsPerCentimeter); 
  world               = new FWorld();
  
  
  /* creation of maze walls */
  
  //horizontal walls
  w1                   = new FBox(9.60, 2);
  w1.setPosition(edgeTopLeftX+worldWidth/2.0, edgeTopLeftY+2*worldHeight/3.0);
  w1.setFill(0);
  w1.setNoStroke();
  w1.setStaticBody(true);
  //wall.setFill(0, 155, 200);
  world.add(w1);
  
  w2                   = new FBox(10, 0.5);
  w2.setPosition(edgeTopLeftX+worldWidth/2.0+2, edgeTopLeftY+worldHeight/2.0);
  w2.setFill(0);
  w2.setNoStroke();
  w2.setStaticBody(true);
  //wall.setFill(0, 155, 200);
  world.add(w2);
  //print(wall);
  
  w3                   = new FBox(11, 0.5);
  w3.setPosition(edgeTopLeftX+worldWidth/2.0+1.75, edgeTopLeftY+worldHeight/2.0-5);
  w3.setFill(0);
  w3.setNoStroke();
  w3.setStaticBody(true);
  //wall.setFill(0, 155, 200);
  world.add(w3);

  
  w4                   = new FBox(10, 2);
  w4.setPosition(edgeTopLeftX+worldWidth/2.0, edgeTopLeftY+worldHeight/2.0-2.5);
  w4.setFill(0);
  w4.setNoStroke();
  w4.setStaticBody(true);
  //wall.setFill(0, 155, 200);
  world.add(w4);
   

  //vertical walls
  w5                   = new FBox(0.5, 11);
  w5.setPosition(edgeTopLeftX+worldWidth/3.0, edgeTopLeftY+worldHeight/2.0+.25);
  w5.setFill(0);
  w5.setNoStroke();
  w5.setStaticBody(true);
  //wall.setFill(0, 155, 200);
  world.add(w5);
  print(w5);

  w6                   = new FBox(0.5, 9.5);
  w6.setPosition(edgeTopLeftX+worldWidth/3.0+12, edgeTopLeftY+worldHeight/2.0);
  w6.setFill(0);
  w6.setNoStroke();
  w6.setStaticBody(true);
  //wall.setFill(0, 155, 200);
  world.add(w6);
  print(w6);
  
  w7                   = new FBox(16.5, 0.5);
  w7.setPosition(edgeTopLeftX+worldWidth/2.0+3, edgeTopLeftY+worldHeight/2.0+6);
  w7.setFill(0);
  w7.setNoStroke();
  w7.setStaticBody(true);
  //wall.setFill(0, 155, 200);
  world.add(w7);
  
  w8                  = new FBox(15, 0.5);
  w8.setPosition(edgeTopLeftX+worldWidth/2.0+3.75, edgeTopLeftY+worldHeight/2.0+4.75);
  w8.setFill(0);
  w8.setNoStroke();
  w8.setStaticBody(true);
  //wall.setFill(0, 155, 200);
  world.add(w8);

  
  /* Set viscous layer */
  l1                  = new FBox(28.5,4);
  l1.setPosition(edgeTopLeftX+worldWidth/2,edgeTopLeftY+worldHeight/2.0+5);
  //l1.setFill(150,150,255,80);
  l1.setNoStroke();
  l1.setDensity(100);
  l1.setSensor(true);
  l1.setNoStroke();
  l1.setStatic(true);
  l1.setName("Water");
  world.add(l1);
  
  /* Start Button */
  c1                  = new FCircle(2.0); // diameter is 2
  c1.setPosition(edgeTopLeftX+2, edgeTopLeftY+worldHeight/2.0-5.5);
  c1.setFill(0, 200, 50);
  //c1.setNoStroke();
  c1.setStaticBody(true);
  world.add(c1);
  
  /* Reset Button */
  //special danger pixels
  r1                   = new FBox(1, 1);
  r1.setPosition(edgeTopLeftX+worldWidth/3.0+11.25, edgeTopLeftY+worldHeight/2.0+0.75);
  r1.setStaticBody(true);
  r1.setNoStroke();
  //r1.setFill(200,50,50);
  world.add(r1);
  
  r2                   = new FBox(1, 1);
  r2.setPosition(edgeTopLeftX+worldWidth/3.0+11.25, edgeTopLeftY+worldHeight/2.0+4);
  r2.setStaticBody(true);
  r2.setNoStroke();
  //r2.setFill(200,50,50);
  world.add(r2);
  
  r3                  = new FBox(1, 1);
  r3.setPosition(edgeTopLeftX+worldWidth/3.0+0.7, edgeTopLeftY+worldHeight/2.0-1);
  r3.setStaticBody(true);
  r3.setNoStroke();
  //r3.setFill(200,50,50);
  world.add(r3);
    
  r4                  = new FBox(1, 1);
  r4.setPosition(edgeTopLeftX+worldWidth/3.0+11.25, edgeTopLeftY+worldHeight/2.0-4.25);
  r4.setStaticBody(true);
  r4.setNoStroke();
  //r4.setFill(200,50,50);
  world.add(r4);
  
  //finish block
  finish                   = new FBox(1, 0.8);
  finish.setPosition(edgeTopLeftX+worldWidth/3.0+15.74, edgeTopLeftY+worldHeight/2.0+5.35);
  finish.setStaticBody(true);
  finish.setNoStroke();
  //f.setFill(0,50,50);
  world.add(finish);
  
  //c2                  = new FCircle(2.0);
  //c2.setPosition(worldWidth-2.5, edgeTopLeftY+worldHeight/2.0);
  //c2.setFill(200,0,0);
  //c2.setStaticBody(true);
  //c2.setSensor(true);
  //world.add(c2);
  

    
  /* Haptic Tool Initialization */
  //s                   = new HVirtualCoupling((1)); 
  //s.h_avatar.setDensity(4);  
  //s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 
  
  s                   = new HVirtualCoupling((0.75)); 
  s.h_avatar.setDensity(4); 
  s.h_avatar.setFill(0,0,0); 
  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+worldHeight/2.0-5.5); 
  s.h_avatar.setSensor(true);

  
  /* If you are developing on a Mac users must update the path below 
   * from "../img/Haply_avatar.png" to "./img/Haply_avatar.png" 
   */
  //haplyAvatar = loadImage("../img/Haply_avatar.png"); 
  //haplyAvatar.resize((int)(hAPI_Fisica.worldToScreen(1)), (int)(hAPI_Fisica.worldToScreen(1)));
  //s.h_avatar.attachImage(haplyAvatar); 


  /* world conditions setup */
  world.setGravity((0.0), (1000.0)); //1000 cm/(s^2)
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(.4);
  world.setEdgesFriction(0.5);
  
  
  world.draw();
  
  
  /* setup framerate speed */
  frameRate(baseFrameRate);
  

  /* setup simulation thread to run at 1kHz */ 
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}
/* end setup section ***************************************************************************************************/



/* draw section ********************************************************************************************************/
void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  if(renderingForce == false){
    background(255);
    textFont(f, 22);
 
    if(gameStart){
      fill(0, 0, 0);
      textAlign(CENTER);
      text("Solve the maze while avoiding all the red blocks!!!", width/2, 60);
      //textAlign(CENTER);
      //text("Touch the green circle to reset", width/2, 90);
    
      w1.setFill(0, 155, 200);
      w2.setFill(0, 155, 200);
      w3.setFill(0, 155, 200);
      w4.setFill(0, 155, 200);
      w5.setFill(0, 155, 200);
      w6.setFill(0, 155, 200);
      w7.setFill(0, 155, 200);
      w8.setFill(0, 155, 200);
      
      r1.setFill(200,50,50);
      r2.setFill(200,50,50);
      r3.setFill(200,50,50);
      r4.setFill(200,50,50);
      
      finish.setFill(0,50,50);
      l1.setFill(150,150,255,80);
      
      //w1.setFill(0, 0, 0);
      textAlign(CENTER);
      text("FINISH", width-100, 545);
    
    }
    else{
      fill(128, 128, 128);
      textAlign(CENTER);
      text("Move the cursor to get the game ball!!", width/2, 60);
      textAlign(CENTER);
      text("Touch the green circle to start the maze", width/2, 85);
      
    
      w1.setFill(255, 255, 255);
      w2.setFill(255, 255, 255);
      w3.setFill(255, 255, 255);
      w4.setFill(255, 255, 255);
      w5.setFill(255, 255, 255);
      w6.setFill(255, 255, 255);
      w7.setFill(255, 255, 255);
      w8.setFill(255, 255, 255);
      
      r1.setFill(255, 255, 255);
      r2.setFill(255, 255, 255);
      r3.setFill(255, 255, 255);
      r4.setFill(255, 255, 255);
      
      finish.setFill(255, 255, 255);
      l1.setFill(255, 255, 255,80);
 
    }
  
    world.draw();
  }
}
/* end draw section ****************************************************************************************************/



/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable{
  
  public void run(){
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    
    renderingForce = true;
    
    if(haplyBoard.data_available()){
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();
    
      angles.set(widgetOne.get_device_angles()); 
      posEE.set(widgetOne.get_device_position(angles.array()));
      posEE.set(posEE.copy().mult(200));  
    }
    
    s.setToolPosition(edgeTopLeftX+worldWidth/2-(posEE).x, edgeTopLeftY+(posEE).y-7); 
    s.updateCouplingForce();
    
    fEE.set(-s.getVirtualCouplingForceX(), s.getVirtualCouplingForceY());
    fEE.div(100000); //dynes to newtons
    
    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();
  
    if (s.h_avatar.isTouchingBody(c1)){
      gameStart = true;
      //g1.setPosition(2,8);
      //g2.setPosition(3,8);
      s.h_avatar.setSensor(false);
    }
    
    //reset game if red block touched
    if(s.h_avatar.isTouchingBody(r1)||s.h_avatar.isTouchingBody(r2)||
    s.h_avatar.isTouchingBody(r3)||s.h_avatar.isTouchingBody(r4)||
    s.h_avatar.isTouchingBody(finish)){
      gameStart = false;
      s.h_avatar.setSensor(true);
    }
  
      /* Viscous layer codes */
    if (s.h_avatar.isTouchingBody(l1) && gameStart== true){
      s.h_avatar.setDamping(800);
    }
    else{
      s.h_avatar.setDamping(10); 
    }
  
  
    world.step(1.0f/1000.0f);
  
    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/



/* helper functions section, place helper functions here ***************************************************************/

/* end helper functions section ****************************************************************************************/
