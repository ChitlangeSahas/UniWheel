import processing.serial.*;
Serial serial;

String stringKalmanX, stringKalmanY, stringPIDOutput;

final int width = 800;
final int height = 600;

float[] kalmanX = new float[width];
float[] kalmanY = new float[width];

boolean drawValues  = false;

void setup() {
  size(800, 600);
  println(Serial.list()); // Use this to print connected serial devices
  serial = new Serial(this, Serial.list()[0], 115200); // Set this to your serial port obtained using the line above
  serial.bufferUntil('\n'); // Buffer until line feed

  for (int i = 0; i < width; i++) { // center all variables
    kalmanX[i] = height/2;
    kalmanY[i] = height/2;
  }
}

void draw() {
  /* Draw Graph */
  if (drawValues) {
    drawValues = false;
    drawGraph();
  }
}

void drawGraph() {
  background(0); // White
  //for (int i = 0; i < width; i++) {
  //  stroke(200); // Grey
  //  line(i*10, 0, i*10, height);
  //  line(0, i*10, width, i*10);
  //}

  stroke(255,255,0); // White
  for (int i = 1; i <= 3; i++)
    line(0, height/4*i, width, height/4*i); // Draw line, indicating -90 deg, 0 deg and 90 deg

  convert();
  drawAxisX();
  drawAxisY();
  textSize(17);
  fill(255);
  text("IMU Kalman filter\nGreen: KalmanX\nRed: KalmanY",20.0,30.0);
  text("KalmanX :" + stringKalmanX,20.0,110);
  text("KalmanY :" + stringKalmanY,20.0,130);
  text("PID Output :" + stringPIDOutput, 20.0, 150);
  
  stroke(255);
  translate(width/2 , height/2 - 50);
  int rect_width = 100;
  int rect_hieght = 10;
  fill(0,0,255);
  rotate(radians(float(stringKalmanX)));
  rect(-rect_width/2,-rect_hieght/2,rect_width,rect_hieght);
}

void serialEvent (Serial serial) {
  // Get the ASCII strings:
  stringKalmanX = serial.readStringUntil('\t');
  stringKalmanY = serial.readStringUntil('\t');
  stringPIDOutput = serial.readStringUntil('\t');
  serial.clear(); // Clear buffer
  drawValues = true; // Draw the graph
  //printAxis(); // Used for debugging
}

void printAxis() {
  print(stringKalmanX);
  print('\t');
  print(stringKalmanY);
  println();
}
