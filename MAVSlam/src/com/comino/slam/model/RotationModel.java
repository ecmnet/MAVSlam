package com.comino.slam.model;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import com.comino.msp.utils.MSPMathUtils;

import georegression.geometry.ConvertRotation3D_F32;
import georegression.struct.EulerType;
import georegression.struct.se.Se3_F64;

public class RotationModel {

	public static int X 	= 0;
	public static int Y		= 1;
	public static int Z   	= 2;

	public static int PITCH = 0;
	public static int YAW   = 1;
	public static int ROLL  = 2;

	public int 		  		quality = 0;

	public DenseMatrix64F  	R_NED  	= new DenseMatrix64F(3,3);  // Rotation BODY to NED
	public DenseMatrix64F  	R_BODY  = new DenseMatrix64F(3,3);  // Rotation NED to BODY
	public DenseMatrix64F  	R_VIS   = new DenseMatrix64F(3,3);	// Rotation VisualFrame to NED (init)

	public DenseMatrix64F   R_POS  = new DenseMatrix64F(3,3);	// Rotation CameraPosition to NED


	public void setNED(float[] rotation) {
		ConvertRotation3D_F32.eulerToMatrix(EulerType.XYZ, rotation[X], rotation[Y], rotation[Z], R_NED);
		CommonOps.invert(R_NED, R_BODY);
	}


	public void setVIS(float[] rotation) {
		// Why not negative
		ConvertRotation3D_F32.eulerToMatrix(EulerType.XYZ, rotation[X], rotation[Y], rotation[Z], R_VIS);
	}


	public void setPOS(DenseMatrix64F rpos) {
	    CommonOps.mult(1, rpos, R_VIS, R_POS);
	}

	public static String toString(float[] att) {
		String s = new String();
		if(att.length==3) {
			 s = String.format("Roll=%.1f° Pitch=%.1f° Yaw=%.1f°",
					 MSPMathUtils.fromRad(att[ROLL]),
					 MSPMathUtils.fromRad(att[PITCH]),
					 MSPMathUtils.fromRad(att[YAW]));
		}
		return s;
	}




}
