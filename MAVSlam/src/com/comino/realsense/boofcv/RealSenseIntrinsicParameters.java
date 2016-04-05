package com.comino.realsense.boofcv;

import com.comino.librealsense.wrapper.LibRealSenseWrapper.rs_intrinsics;

import boofcv.struct.calib.IntrinsicParameters;

public class RealSenseIntrinsicParameters extends IntrinsicParameters {

	private static final long serialVersionUID = -3525224116201930353L;

	public RealSenseIntrinsicParameters(rs_intrinsics intrinsics) {


       this.cx = intrinsics.ppx;
       this.cy = intrinsics.ppy;

       this.width  = intrinsics.width;
       this.height = intrinsics.height;

       this.fx = intrinsics.fx;
       this.fy = intrinsics.fy;

	}

	public String toString() {
		return "cx="+cx+" cy="+cy+" fx="+fx+" fy="+fy;
	}

}
