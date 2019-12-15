//convert all axis
final int minAngle = -180;
final int maxAngle = 180;

void convert() {
  /* Convert the kalman filter x-axis */
  if (stringKalmanX != null) {
    stringKalmanX = trim(stringKalmanX); // Trim off any whitespace
    kalmanX[kalmanX.length - 1] = map(float(stringKalmanX), minAngle, maxAngle, 0, height); // Convert to a float and map to the screen height, then save in buffer
  }

  /* Convert the kalman filter y-axis */
  if (stringKalmanY != null) {
    stringKalmanY = trim(stringKalmanY); // Trim off any whitespace
    kalmanY[kalmanY.length - 1] = map(float(stringKalmanY), minAngle, maxAngle, 0, height); // Convert to a float and map to the screen height, then save in buffer
  }
}
