/**
 **********************************************************************************************************************
 * @file       sketch_2_Hello_Wall.pde
 * @author     Steve Ding, Colin Gallacher, Antoine Weill--Duflos
 * @version    V1.0.0
 * @date       09-February-2021
 * @brief      PID example with random position of a target
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
import controlP5.*;
/* end library imports *************************************************************************************************/  


/* scheduler definition ************************************************************************************************/ 
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/ 

ControlP5 cp5;



/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 5;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           renderingForce                     = false;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 



/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerMeter                      = 4000.0;
float             radsPerDegree                       = 0.01745;

/* pantagraph link parameters in meters */
float             l                                   = 0.07;
float             L                                   = 0.09;


/* end effector radius in meters */
float             rEE                                 = 0.006;


/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);
PVector           oldangles                              = new PVector(0, 0);
PVector           diff                              = new PVector(0, 0);


/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                 = new PVector(0, 0); 

/* device graphical position */
PVector           deviceOrigin                        = new PVector(0, 0);

/* World boundaries reference */
final int         worldPixelWidth                     = 1200;
final int         worldPixelHeight                    = 650;

float x_m,y_m;

// used to compute the time difference between two loops for differentiation
long oldtime = 0;
// for changing update rate
int iter = 0;

/// PID stuff

float P = 0.0;
// for I
float I = 0;
float cumerrorx = 0;
float cumerrory = 0;
// for D
float oldex = 0.0f;
float oldey = 0.0f;
float D = 0;

//for exponential filter on differentiation
float diffx = 0;
float diffy = 0;
float buffx = 0;
float buffy = 0;
float smoothing = 0.80;

float xr = 0;
float yr = 0;
float y_pos =400;
float x_pos=1000;

// checking everything run in less than 1ms
long timetaken= 0;

// set loop time in usec (note from Antoine, 500 is about the limit of my computer max CPU usage)
int looptime = 500;


/* graphical elements */
PShape pGraph, joint, endEffector;
PShape wall;
PShape target;
PShape path;
PFont f;

float[] fsr;
float knobval;
Knob knobP;
Knob knobI;
Knob knobD;
Slider sliderL;
float theta = 0.0;
float speed;
boolean follow =false;

int m;
int counter =1;
int seconds;
int timeS;
int oldtimes;
float timeelapsed= 0.0;
ControlP5 controlP5;
Slider c3;
Textlabel FSRtxt;
Textlabel tasktxt;
Textlabel timertxt;
PShape targetFSR;
int from = color(255, 0, 0);
int to = color(0, 120, 200);

/* end elements definition *********************************************************************************************/ 




/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(1200, 700);
  
  /* GUI setup */
    smooth();
  cp5 = new ControlP5(this);
  /*
  knobP=cp5.addKnob("P")
               .setRange(0,2000)
               .setValue(0)
               .setPosition(50,250)
               .setRadius(100)
               .setDragDirection(Knob.VERTICAL)
               ; 
               */
    FSRtxt=cp5.addTextlabel("Prop")
                    .setText(str(theta))
                    .setPosition(1050,20)
                    .setColorValue(color(170,20,20))
                    .setFont(createFont("Georgia",30))
                    ;
                    
    tasktxt=cp5.addTextlabel("Task")
                    .setText("Task 1/10")
                    .setPosition(600,20)
                    .setColorValue(color(170,20,20))
                    .setFont(createFont("Georgia",40))
                    ;
                    
      timertxt=cp5.addTextlabel("Time")
                    .setText(str(theta))
                    .setPosition(750,600)
                    .setColorValue(color(0,255,100))
                    .setFont(createFont("Georgia",40))
                    ;
         
  cp5.addButton("ResetDevice")
     .setValue(0)
     .setPosition(10,620)
     .setSize(200,50)
     ;
     
  target = createShape(ELLIPSE, 0,0, 100, 100);
  target.setFill(false);
  
  
  //section for gradient   
  controlP5 = new ControlP5(this);

  //change the original colors
  controlP5.setColorForeground(lerpColor(from, to, 0.5));
  controlP5.setColorBackground(color(150, 158, 159));
  controlP5.setColorActive(lerpColor(from, to, 0.5));


  c3=controlP5.addSlider("grad2")
    .setRange(0, 2000)
    .setValue(20)
    .setPosition(1000, 80)
    .setSize(120, 600)
    .setColorValue(200)
    .setColorLabel(200);

  // draw controls manually so that you can draw on top of them
  controlP5.setAutoDraw(false);
     
  path = createShape(RECT, 900,0, 900, 700);
  path.setStroke(color(50));
  path.setFill(color(0,0,0));

  targetFSR = createShape(RECT, 0,0, 120, 60);
  targetFSR.setStroke(color(0));
  targetFSR.setFill(false);
  //targetFSR.strokeWeight(5);
  //targetFSR.setFill(color(0,0,0));
  

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
  haplyBoard          = new Board(this, "COM3", 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();
  
  widgetOne.set_mechanism(pantograph);
  
  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);
 
  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);
  widgetOne.add_analog_sensor("A2");  
  widgetOne.device_set_parameters();
    
  
  /* visual elements setup */
  background(0);
  deviceOrigin.add(worldPixelWidth/2-200, 0);
  
  /* create pantagraph graphics */
  create_pantagraph();
  
  

  
  /* setup framerate speed */
  frameRate(baseFrameRate);
    f = createFont("Arial",16,true); // STEP 2 Create Font
  
  /* setup simulation thread to run at 1kHz */ 
  thread("SimulationThread");
}
/* end setup section ***************************************************************************************************/

public void RandomPosition(int theValue) {
    xr = random(-0.5,0.5);
    yr = random(-0.5,0.5);
    y_pos = random(180,500);
    counter=counter+1;
}
public void ResetIntegrator(int theValue) {
    cumerrorx= 0;
    cumerrory= 0;
}
public void ResetDevice(int theValue) {
    widgetOne.device_set_parameters();

}


/* Keyboard inputs *****************************************************************************************************/

/// Antoine: this is specific to qwerty keyboard layout, you may want to adapt

void keyPressed() {
}

/* draw section ********************************************************************************************************/
void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
   m=millis();
   seconds = m/1000;
  if(renderingForce == false){
    background(255); 
    shape(path);
    path.setFill(color(0,0,0));
    
    update_animation(angles.x*radsPerDegree, angles.y*radsPerDegree, posEE.x, posEE.y);
    if (follow){//pathtracking(looptime);     
    }
  }
}
/* end draw section ****************************************************************************************************/
void slideGradient(Controller c) {
  float[] p = c.getPosition();
  float amt = c.getValue()/(c.getMax()-c.getMin());
  slideGradient(int(p[0]), int(p[1]), c.getWidth(), c.getHeight(), from, to, amt);
}

void slideGradient(int x, int y, float w, float h, color c1, color c2, float amt) {
  int pos = int((y+h)-(h*amt));
  for (int i = int(y+h); i >= pos; i--) {
    float inter = map(i, y, y+h, 0, 1);
    color c = lerpColor(c1, c2, inter);
    stroke(c);
    line(x, i, x+w, i);
  }
}
/* end draw section ****************************************************************************************************/

int noforce = 0;
long timetook = 0;
long looptiming = 0;
/* simulation section **************************************************************************************************/
public void SimulationThread(){
while(1==1) {
    long starttime = System.nanoTime();
    long timesincelastloop=starttime-timetaken;
    iter+= 1;
    // we check the loop is running at the desired speed (with 10% tolerance)
    if(timesincelastloop >= looptime*1000*1.1) {
      float freq = 1.0/timesincelastloop*1000000.0;
        //println("caution, freq droped to: "+freq + " kHz");
        //println("P " + P +" I  " + I + " D " + D + " smoothing = " + smoothing + " Looptime " + looptime);
    }
    else if(iter >= 1000) {
      float freq = 1000.0/(starttime-looptiming)*1000000.0;
       //println("loop running at "  + freq + " kHz");
       //println("P " + P +" I  " + I + " D " + D + " smoothing = " + smoothing + " Looptime " + looptime);
       iter=0;
       looptiming=starttime;
    }
    
    timetaken=starttime;
    
    renderingForce = true;
    
    if(haplyBoard.data_available()){
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();
      fsr= widgetOne.get_sensor_data();    
      //for (int i = 0; i < fsr.length; i++)
      //print(" " + fsr[0]);
      knobval=fsr[0];
      //println();
      //knobP.setValue(knobval);
      c3.setValue(knobval);  
      FSRtxt.setText(str(knobval));
      int s = second(); 
      float range_fsr=abs(knobval-((675-y_pos)*(2000/600)));
      //println(" " + knobval + " " + (675-y_pos)*(2000/600)); 
      
      if (seconds>timeS){
      timeS=seconds;
      println(timeS); 
      }
      
      if (range_fsr<130){
      timertxt.setText("Time: "+ str(timeS-oldtimes));
      targetFSR.setStrokeWeight(5);
            if ((timeS-oldtimes)>4){      
            targetFSR.setStroke(color(0,255,0)); 
            targetFSR.setStrokeWeight(6);
            if ((timeS-oldtimes)>5){ 
            RandomPosition(1);
            tasktxt.setText("Task " + str(counter)+"/10");            
             }
            }
      }
      else{
      timertxt.setText("0");
      oldtimes=timeS;
      targetFSR.setStrokeWeight(3);
      targetFSR.setStroke(color(0,0,0)); 
      }
      
      
      noforce = 0;
      angles.set(widgetOne.get_device_angles());
    
      posEE.set(widgetOne.get_device_position(angles.array()));

      posEE.set(device_to_graphics(posEE)); 
      x_m = xr*300; 
      y_m = yr*300+350;//mouseY;
      
 // Torques from difference in endeffector and setpoint, set gain, calculate force
      float xE = pixelsPerMeter * posEE.x;
      float yE = pixelsPerMeter * posEE.y;
      long timedif = System.nanoTime()-oldtime;

      float dist_X = x_m-xE;
      cumerrorx += dist_X*timedif*0.000000001;
      float dist_Y = y_m-yE;
      cumerrory += dist_Y*timedif*0.000000001;
      //println(dist_Y*k + " " +dist_Y*k);
      // println(timedif);
      if(timedif > 0) {
        buffx = (dist_X-oldex)/timedif*1000*1000;
        buffy = (dist_Y-oldey)/timedif*1000*1000;            

        diffx = smoothing*diffx + (1.0-smoothing)*buffx;
        diffy = smoothing*diffy + (1.0-smoothing)*buffy;
        oldex = dist_X;
        oldey = dist_Y;
        oldtime=System.nanoTime();
      }
    
    // Forces are constrained to avoid moving too fast
  
      fEE.x = constrain(P*dist_X,-4,4) + constrain(I*cumerrorx,-4,4) + constrain(D*diffx,-8,8);

      
      fEE.y = constrain(P*dist_Y,-4,4) + constrain(I*cumerrory,-4,4) + constrain(D*diffy,-8,8); 


      if(noforce==1)
      {
        fEE.x=0.0;
        fEE.y=0.0;
      }
    widgetOne.set_device_torques(graphics_to_device(fEE).array());
    //println(f_y);
      /* end haptic wall force calculation */
      
    }
    
    
    
    widgetOne.device_write_torques();
  
  
    renderingForce = false;
    long timetook=System.nanoTime()-timetaken;
    if(timetook >= 1000000) {
    //println("Caution, process loop took: " + timetook/1000000.0 + "ms");
    }
    else {
      while(System.nanoTime()-starttime < looptime*1000) {
      //println("Waiting");
      }
    }
    
  }
}

/* end simulation section **********************************************************************************************/


/* helper functions section, place helper functions here ***************************************************************/
void create_pantagraph(){
  float lAni = pixelsPerMeter * l;
  float LAni = pixelsPerMeter * L;
  float rEEAni = pixelsPerMeter * rEE;
  
  pGraph = createShape();
  pGraph.beginShape();
  pGraph.fill(255);
  pGraph.stroke(0);
  pGraph.strokeWeight(2);
  
  pGraph.vertex(deviceOrigin.x, deviceOrigin.y);
  pGraph.vertex(deviceOrigin.x, deviceOrigin.y);
  pGraph.vertex(deviceOrigin.x, deviceOrigin.y);
  pGraph.vertex(deviceOrigin.x, deviceOrigin.y);
  pGraph.endShape(CLOSE);
  
  joint = createShape(ELLIPSE, deviceOrigin.x, deviceOrigin.y, rEEAni, rEEAni);
  joint.setStroke(color(0));
  
  endEffector = createShape(ELLIPSE, deviceOrigin.x, deviceOrigin.y, 2*rEEAni, 2*rEEAni);
  endEffector.setStroke(color(0));
  strokeWeight(5);
  
}


PShape create_wall(float x1, float y1, float x2, float y2){
  x1 = pixelsPerMeter * x1;
  y1 = pixelsPerMeter * y1;
  x2 = pixelsPerMeter * x2;
  y2 = pixelsPerMeter * y2;
  
  return createShape(LINE, deviceOrigin.x + x1, deviceOrigin.y + y1, deviceOrigin.x + x2, deviceOrigin.y+y2);
}




void update_animation(float th1, float th2, float xE, float yE){
  //background(255);
    pushMatrix();
  float lAni = pixelsPerMeter * l;
  float LAni = pixelsPerMeter * L;
  
  xE = pixelsPerMeter * xE;
  yE = pixelsPerMeter * yE;
  
  th1 = 3.14 - th1;
  th2 = 3.14 - th2;
    
  pGraph.setVertex(1, deviceOrigin.x + lAni*cos(th1), deviceOrigin.y + lAni*sin(th1));
  pGraph.setVertex(3, deviceOrigin.x + lAni*cos(th2), deviceOrigin.y + lAni*sin(th2));
  pGraph.setVertex(2, deviceOrigin.x + xE, deviceOrigin.y + yE);
  
  shape(pGraph);
  shape(joint);
  float[] coord;
  
  
  translate(xE, yE);
  shape(endEffector);
  popMatrix();
  //arrow(xE,yE,fEE.x,fEE.y);
  textFont(f,16);                  // STEP 3 Specify font to be used
  fill(0);                         // STEP 4 Specify font color 
  x_m = xr*300+500; 
  y_m = yr*300+350;//mouseY;
  pushMatrix();
  translate(x_m, y_m);
  float xE_vir=xE+400;
  shape(target);  
  //custom calc
  //println(" " +x_m + " " + xE_vir + " " + y_m + " " + yE);  
  float range_x=abs(xE_vir-x_m);
  float range_y=abs(yE-y_m);
  if((range_x < 25 ) && (range_y < 25)) {
  target.setStroke(color(0,255,0));  
  target.setStrokeWeight(5);
  }
  else{
  target.setStroke(color(0,0,0));  
  target.setStrokeWeight(1);
  }
  
  
  
  
  //println(" " +range_x + " "+ range_y);  
  popMatrix();
  // slider 2: render default with cp5.draw() then draw gradient on top  
  controlP5.draw();
  //Controller c2 = controlP5.getController("grad2");
  slideGradient(c3); 
  pushMatrix();
  translate(1000, y_pos);
  shape(targetFSR);
  targetFSR.setStrokeWeight(3);  
  popMatrix();

}



PVector device_to_graphics(PVector deviceFrame){
  return deviceFrame.set(-deviceFrame.x, deviceFrame.y);
}


PVector graphics_to_device(PVector graphicsFrame){
  return graphicsFrame.set(-graphicsFrame.x, graphicsFrame.y);
}

void arrow(float x1, float y1, float x2, float y2) {
    x2=x2*10.0;
  y2=y2*10.0;
  x1=x1+500;
  x2=-x2+x1;
  y2=y2+y1;

  line(x1, y1, x2, y2);
  pushMatrix();
  translate(x2, y2);
  float a = atan2(x1-x2, y2-y1);
  rotate(a);
  line(0, 0, -10, -10);
  line(0, 0, 10, -10);
  popMatrix();
} 

void pathtracking(float times){
  xr = 0.75*cos(theta);
  yr = 0.5*sin(theta);
  speed=10/times;
  theta = theta+speed;
  if (theta==360)
    theta = 0;  
}
  

/* end helper functions section ****************************************************************************************/




 
