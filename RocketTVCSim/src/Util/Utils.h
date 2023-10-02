#pragma once
inline double degreesToRad(double degrees) {
	//converts degrees to radians
	return degrees * (3.14159265358979323846 / 180);
}
inline double clamp(double inputVal, double maxVal) {
	//Clamps inputVal to between -maxVal and maxVal
	if (inputVal > maxVal) {
		return maxVal;
	}
	else if (inputVal < -maxVal) {
		return -maxVal;
	}
	else {
		return inputVal;
	}
}
