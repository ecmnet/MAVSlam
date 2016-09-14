package com.comino.slam.detectors.space;

import java.util.ArrayList;
import java.util.List;

public class NavigationBlock {

	private List<AttributedPoint3D_F32> features;
	private boolean isObstacle = false;

	public NavigationBlock() {
		this.features = new ArrayList<AttributedPoint3D_F32>();
	}

	public List<AttributedPoint3D_F32> getFeatures() {
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
