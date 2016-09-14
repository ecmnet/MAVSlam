package com.comino.slam.detectors.space;

import boofcv.struct.image.Color3_I32;
import boofcv.struct.image.GrayI;
import georegression.struct.point.Point3D_F32;
import georegression.struct.point.Point3D_I32;

public class AttributedPoint3D_F32 extends Point3D_F32 {

	private static final long serialVersionUID = -2022100770105433425L;

	public Color3_I32 color;

	public AttributedPoint3D_F32(int x, int y, int z, Color3_I32 grey_val) {
		super(x,y,z);
		this.color = color;
	}

}
