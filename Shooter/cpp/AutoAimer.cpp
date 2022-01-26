#include <AutoAimer.h>

AutoAimer::AutoAimer() {
    //TODO: initialize dist_to_settings

}

AutoAimer::Settings AutoAimer::DistanceToSettings(double dist) {
    //TODO: if sensor doesn't see thing, return 0, 0, 0
	//if we have a range finder this class will probably own it

    double dist1, dist2;
    AutoAimer::Settings s;

    if (binary_search(dists, dist, dist1, dist2)) { 
        AutoAimer::Settings s1 = dist_to_settings[dist1];
        AutoAimer::Settings s2 = dist_to_settings[dist2];

        s.flywheel_speed = interpolate(dist, s1.flywheel_speed, s2.flywheel_speed, dist1, dist2);
        s.hood_pose = interpolate(dist, s1.hood_pose, s2.hood_pose, dist1, dist2);
        s.kicker_speed = interpolate(dist, s1.kicker_speed, s2.kicker_speed, dist1, dist2);
    }

    return s;
}

bool AutoAimer::binary_search(std::vector<double> &array, double p, double &p1, double &p2) {
    int left = 0, right = array.size()-1;

	if (p < array[left] || p > array[right]) {
		return false;
	}

	int mid = (left+right)/2;

	while (left < right) {
		if (array[mid] <= p && p <= array[mid+1]) {
			p1 = array[mid];
			p2 = array[mid+1];
			return true;
		} else if (array[mid] <= p) {
			left = mid;
		} else {
			right = mid;
		}
		mid = (left+right)/2;
	}
	return false;
}

double interpolate(double dist, double prev_setting, double next_setting, double prev_dist, double next_dist) {
    return prev_setting + (next_setting - prev_setting)*((dist-prev_dist)/(next_dist-prev_dist));
}