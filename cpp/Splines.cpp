#include "Splines.h"

// Calculate segmentation length (returns -1 if there was an issue)
double Splines::calculateSegLength(int node, Spline spline) {
  double segLength = 0;
  SplinePoint oldPoint, newPoint;
  oldPoint = Splines::getSplinePoint((float)node, spline);

  std::cout << "[Node " << node << "-" << node+1 << "]" << std::endl;
  for (double t = 0.0; t < 1.0; t += _stepSize) {
    newPoint = Splines::getSplinePoint((float)node + t, spline);

    // 1 means end of spline so return
    if (newPoint.flag != 0 || oldPoint.flag != 0) {
      printProgress(1);
      std::cout << " Complete" << std::endl;
      return segLength;
    }

    double xrt = (newPoint.waypoint.x - oldPoint.waypoint.x)*(newPoint.waypoint.x - oldPoint.waypoint.x);
    double yrt = (newPoint.waypoint.y - oldPoint.waypoint.y)*(newPoint.waypoint.y - oldPoint.waypoint.y);
    double xyrt = (xrt+yrt);
    
    double bufferLength = 0;

    if (xyrt > 0) {
      bufferLength = sqrt(xyrt);
      if (isinf(bufferLength) || isnan(bufferLength)) {
        bufferLength = 0;
        std::cout << " -- Overflow detected, Debug Below -- " << std::endl;
        std::cout << "| New points x,y: (" << (double)newPoint.waypoint.x << "," << (double)newPoint.waypoint.y << ")" << std::endl;
        std::cout << "| Old points x,y: (" << oldPoint.waypoint.x << "," << oldPoint.waypoint.y << ")" << std::endl;
        std::cout << "| t value: " << t << std::endl;
        std::cout << "| XY rt was xrt: (" << xrt << ") & yrt: (" << yrt << ")" << std::endl;
        return -1;
      }
    } else {
      bufferLength = 0;
    }

    segLength += bufferLength;
    oldPoint = newPoint;
    printProgress(t);
  }

  return segLength;
}

/**
 * Build spline,
 * The input of the spline is a reference (&) so it changes the input spline rather than
 * creating a new one and outputing it.
 * Instead the function returns an int, -1 or 1 depending on success or failure
 * 
 * Use remove nodes to remove nodes to remove a number of nodes from the front and back of spline
 * (Should be useless because the algorithm does this for you. But there anyway to play around with)
 */
int Splines::buildPath(Spline &spline, int removeNodes) {
  int nodeNum = spline.waypoints.size();

  std::cout << "-- Calculating Length of spline --" << std::endl;
  std::cout << "-- Total Nodes: " << nodeNum << std::endl;
  for (size_t node = removeNodes; node < nodeNum - removeNodes; node++) {
    double segLength = calculateSegLength(node, spline);
    if (segLength == -1) {
      std::cout << "Segment Length Error" << std::endl;
      return -1;
    } else {

      spline.waypoints[node].segLength = segLength;
      spline.totalLength += segLength;
      spline.waypoints[node].totalLength = spline.totalLength;
      if (spline.waypoints[node].segLength > 0) {
        spline.segmentNum++;
        std::cout << "Segment " << node << "-" << node+1 << ", Length: " << spline.waypoints[node].segLength << ", Length up to and including: " << spline.waypoints[node].totalLength << std::endl; 
      }
    }
  }

  std::cout << "Number of segments: " << spline.segmentNum << std::endl;
  splineLength = spline.totalLength;
  // std::cout << "\n\nTotal Length: " << splineLength << std::endl;
  return 0;
}

void Splines::printProgress(double percentage) {
  int val = (int) (percentage * 100);
  int lpad = (int) (percentage * PBWIDTH);
  int rpad = PBWIDTH - lpad;
  printf("\r%3d%% [%.*s%*s]", val, lpad, PBSTR, rpad, "");
  fflush(stdout);
}

Splines::SplinePoint Splines::getSplinePoint(float t, Spline spline) {
  int p0, p1, p2, p3;

  p1 = (int)t + 1;
  p2 = p1 + 1;
  p3 = p2 + 1;
  p0 = p1 - 1;

  if (p3 >= spline.waypoints.size()) {
    return {{0,0}, 1}; // out of scope (return segment complete -> 1)
  }

  t = t - (int)t;

  float tt = t * t;
  float ttt = tt * t;

  // float q1 = -ttt + 2.0f*tt - t;
  // float q2 = 3.0f*ttt - 5.0f*tt + 2.0f;
  // float q3 = -3.0f*ttt + 4.0f*tt + t;
  // float q4 = ttt - tt;

  float q1 = 2.0f*ttt - 3.0f*tt + 1;
  float q2 = ttt - 2.0f*tt + t;
  float q3 = -2.0f*ttt + 3.0f*tt;
  float q4 = ttt - tt;

  float tx =  (spline.waypoints[p0].x * q1 + spline.waypoints[p1].x * q2 + spline.waypoints[p2].x * q3 + spline.waypoints[p3].x * q4);
  float ty =  (spline.waypoints[p0].y * q1 + spline.waypoints[p1].y * q2 + spline.waypoints[p2].y * q3 + spline.waypoints[p3].y * q4);
  
  // Detect if it's actually a number and isn't INF or NaN
  if (isinf(tx) || isinf(ty) || isnan(tx) || isnan(ty)) {
    return {{0,0}, -1};
  } else {
    return{ tx, ty };
  }
}

Splines::SplinePoint Splines::getSplineGradientPoint(float t, Spline spline) {
  int p0, p1, p2, p3;
  p1 = (int)t + 1;
  p2 = p1 + 1;
  p3 = p2 + 1;
  p0 = p1 - 1;

  if (p3 >= spline.waypoints.size()) {
    return {{0,0}, 1}; // out of scope (return segment complete -> 1)
  }

  t = t - (int)t;

  float tt = t * t;
  // float ttt = tt * t;

  // float q1 = -3.0f * tt + 4.0f*t - 1;
  // float q2 = 9.0f*tt - 10.0f*t;
  // float q3 = -9.0f*tt + 8.0f*t + 1.0f;
  // float q4 = 3.0f*tt - 2.0f*t;

  float q1 = 6.0f * tt - 6.0f*t;
  float q2 = 3.0f*tt - 4.0f*t + 1;
  float q3 = 6.0f*t - 6.0f*tt;
  float q4 = 3.0f*tt - 2.0f*t;

  float tx = (spline.waypoints[p0].x * q1 + spline.waypoints[p1].x * q2 + spline.waypoints[p2].x * q3 + spline.waypoints[p3].x * q4);
  float ty = (spline.waypoints[p0].y * q1 + spline.waypoints[p1].y * q2 + spline.waypoints[p2].y * q3 + spline.waypoints[p3].y * q4);

  // Detect if it's actually a number and isn't INF or NaN
  if (isinf(tx) || isinf(ty) || isnan(tx) || isnan(ty)) {
    return {{0,0}, -1};
  } else {
    return{ tx, ty };
  }
}

/**
 * Get angle in radians based from t value and spline
 * returns waypoint and flag
 */
double Splines::getSplineAngleRad(float t, Spline spline) {
  SplinePoint gradientPoint = getSplineGradientPoint(t, spline);
  if (gradientPoint.flag != -1) {
    return atan2(gradientPoint.waypoint.y, gradientPoint.waypoint.x);
  } else {
    return 0;
  }
}

/**
 * Get angle in degrees based from t value and spline
 */
double Splines::getSplineAngleDeg(float t, Spline spline) {
  return (getSplineAngleRad(t, spline) * 180 / M_PI); // convert radians to degrees
}
