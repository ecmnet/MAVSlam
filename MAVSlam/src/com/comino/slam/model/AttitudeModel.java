package com.comino.slam.model;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import georegression.geometry.ConvertRotation3D_F32;
import georegression.struct.EulerType;

public class AttitudeModel {

	public int 		  		quality = 0;

	public DenseMatrix64F  	R_NED  	= new DenseMatrix64F(3,3);
	public DenseMatrix64F  	R_BODY  = new DenseMatrix64F(3,3);
	public DenseMatrix64F  	R_VIS   = new DenseMatrix64F(3,3);


	public void setR(float pitch, float roll, float yaw) {
		ConvertRotation3D_F32.eulerToMatrix(EulerType.XYZ,-pitch, -roll, -yaw, R_NED);
		CommonOps.invert(R_NED, R_BODY);
	}

	public void setV(float pitch, float roll, float yaw) {
		ConvertRotation3D_F32.eulerToMatrix(EulerType.XYZ,-pitch, -roll, -yaw, R_VIS);
	}

}
