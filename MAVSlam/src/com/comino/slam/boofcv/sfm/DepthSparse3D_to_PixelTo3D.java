package com.comino.slam.boofcv.sfm;

import boofcv.abst.sfm.ImagePixelTo3D;
import boofcv.struct.image.ImageGray;

/**
 * Wrapper around {@link DepthSparse3D} for {@link ImagePixelTo3D}.
 *
 * @author Peter Abeles
 */
public class DepthSparse3D_to_PixelTo3D<T extends ImageGray>
	implements ImagePixelTo3D
{
	DepthSparse3D<T> alg;

	public DepthSparse3D_to_PixelTo3D(DepthSparse3D<T> alg) {
		this.alg = alg;
	}

	@Override
	public boolean process(double x, double y) {
		return alg.process((int)x,(int)y);
	}

	@Override
	public double getX() {
		return alg.getWorldPt().x;
	}

	@Override
	public double getY() {
		return alg.getWorldPt().y;
	}

	@Override
	public double getZ() {
		return alg.getWorldPt().z;
	}

	@Override
	public double getW() {
		return 1;
	}
}