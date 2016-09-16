package com.comino.slam.detectors.space;

import boofcv.struct.image.Color3_I32;
import boofcv.struct.image.GrayI;
import georegression.struct.point.Point3D_F32;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Point3D_I32;

public class Feature extends Point3D_F32{

	private static final long serialVersionUID = -2022100770105433425L;

	public Feature(double x, double y, double z) {
		super((float)x,(float)y,(float)z);
	}

}
