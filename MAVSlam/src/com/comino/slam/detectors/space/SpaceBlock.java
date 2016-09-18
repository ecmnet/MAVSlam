package com.comino.slam.detectors.space;

import java.util.ArrayList;
import java.util.List;

public class NavigationBlock {

	private List<Feature> features;
	private boolean isObstacle = false;

	public NavigationBlock() {
		this.features = new ArrayList<Feature>();
	}

	public List<Feature> getFeatures() {
		return features;
	}

	public void markAsObstacle(boolean obstacle) {
		isObstacle = obstacle;
	}

	public boolean isObstacle() {
		return isObstacle;
	}

	public void clear() {
		isObstacle = false;
		features.clear();
	}

}
