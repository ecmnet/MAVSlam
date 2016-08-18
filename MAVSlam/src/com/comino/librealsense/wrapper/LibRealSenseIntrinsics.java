package com.comino.librealsense.wrapper;

import com.comino.librealsense.wrapper.LibRealSenseWrapper.rs_intrinsics;

import boofcv.struct.calib.IntrinsicParameters;

public class LibRealSenseIntrinsics extends IntrinsicParameters {

	private static final long serialVersionUID = -3525224116201930353L;

	public int model=0;

	public LibRealSenseIntrinsics(rs_intrinsics intrinsics) {

	   this.model = intrinsics.model;

       this.cx = intrinsics.ppx;
       this.cy = intrinsics.ppy;

       this.width  = intrinsics.width;
       this.height = intrinsics.height;

       this.fx = intrinsics.fx;
       this.fy = intrinsics.fy;

       this.radial = new double[5];
       for(int i=0;i<intrinsics.coeffs.length;i++)
    	   this.radial[i] = intrinsics.coeffs[i];

       this.t1 = 0;
       this.t2 = 0;



	}

	public String toString() {
		return "cx="+cx+" cy="+cy+" fx="+fx+" fy="+fy;
	}

}
