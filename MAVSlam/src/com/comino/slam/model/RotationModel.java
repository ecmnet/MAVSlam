package com.comino.slam.model;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import georegression.geometry.ConvertRotation3D_F32;
import georegression.geometry.GeometryMath_F64;
import georegression.struct.EulerType;

public class RotationModel {

	public static int ROLL 	= 0;
	public static int PITCH = 1;
	public static int YAW   = 2;

	public int 		  		quality = 0;

	public DenseMatrix64F  	R_NED  	= new DenseMatrix64F(3,3);  // Rotation BODY to NED
	public DenseMatrix64F  	R_BODY  = new DenseMatrix64F(3,3);  // Rotation NED to BODY
	public DenseMatrix64F  	R_VIS   = new DenseMatrix64F(3,3);	// Rotation VisualFrame to NED

	public DenseMatrix64F   R_POS  = new DenseMatrix64F(3,3);	// Rotation CameraPosition to NED


	public void setNED(float[] rotation) {
		ConvertRotation3D_F32.eulerToMatrix(EulerType.XYZ, rotation[ROLL], rotation[PITCH], rotation[YAW], R_NED);
		CommonOps.invert(R_NED, R_BODY);
	}

	public void setNED(float pitch, float roll, float yaw) {
		ConvertRotation3D_F32.eulerToMatrix(EulerType.XYZ,roll,pitch,yaw, R_NED);
		CommonOps.invert(R_NED, R_BODY);
	}

	public void setVIS(float[] rotation) {
		// Why not negative
		ConvertRotation3D_F32.eulerToMatrix(EulerType.XYZ, rotation[ROLL], rotation[PITCH], rotation[YAW], R_VIS);
	}

	public void setVIS(float pitch, float roll, float yaw) {
		// Why not negative
		ConvertRotation3D_F32.eulerToMatrix(EulerType.XYZ, roll, pitch, yaw, R_VIS);
	}

	public void setPOS(DenseMatrix64F rpos) {
		CommonOps.mult(1, rpos, R_VIS,R_POS);
	}




}
