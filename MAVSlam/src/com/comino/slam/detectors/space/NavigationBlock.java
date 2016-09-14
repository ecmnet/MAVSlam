package com.comino.slam.detectors.space;

import java.util.ArrayList;
import java.util.List;

public class NavigationBlock {

	private List<AttributedPoint3D_F32> features;

	public NavigationBlock() {
		this.features = new ArrayList<AttributedPoint3D_F32>();
	}

	public List<AttributedPoint3D_F32> getFeatures() {
		return features;
	}

}
