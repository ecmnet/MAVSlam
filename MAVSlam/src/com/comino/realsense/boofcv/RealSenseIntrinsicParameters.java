package com.comino.realsense.boofcv;

import com.comino.librealsense.wrapper.LibRealSenseWrapper.rs_intrinsics;

import boofcv.struct.calib.IntrinsicParameters;

public class RealSenseIntrinsicParameters extends IntrinsicParameters {

	private static final long serialVersionUID = -3525224116201930353L;

	public RealSenseIntrinsicParameters(rs_intrinsics intrinsics) {

		System.out.println(intrinsics);

	    for(int i=0; i<intrinsics.coeffs.length;i++) {
	    	System.out.println(intrinsics.coeffs[i]);
	    }

       this.cx = intrinsics.width / 2;
       this.cy = intrinsics.height / 2;

       this.width  = intrinsics.width;
       this.height = intrinsics.height;

       this.fx = intrinsics.fx;
       this.fy = intrinsics.fy;


	}

}
