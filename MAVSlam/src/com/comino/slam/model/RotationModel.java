package com.comino.slam.model;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import com.comino.msp.utils.MSPMathUtils;

import georegression.geometry.ConvertRotation3D_F64;
import georegression.struct.EulerType;
import georegression.struct.point.Vector3D_F64;

public class RotationModel {

	public static int ROLL  = 0;
	public static int PITCH = 1;
	public static int YAW   = 2;

	public int 		  		quality = 0;

	public DenseMatrix64F  	R_NED  	= new DenseMatrix64F(3,3);  // Rotation BODY to NED
	public DenseMatrix64F  	R_BODY  = new DenseMatrix64F(3,3);  // Rotation NED to BODY
	public DenseMatrix64F  	R_VIS   = new DenseMatrix64F(3,3);	// Rotation VisualFrame to NED (init)

	public DenseMatrix64F   R_POS  = new DenseMatrix64F(3,3);	// Rotation CameraPosition to NED

	public void setNED(double roll, double pitch, double yaw) {
		ConvertRotation3D_F64.eulerToMatrix(EulerType.ZXY, roll, pitch, yaw, R_NED);
		CommonOps.invert(R_NED,R_BODY);
	}

	public void setVIS(double[] rotation) {
		ConvertRotation3D_F64.eulerToMatrix(EulerType.ZXY, -rotation[ROLL], -rotation[PITCH], rotation[YAW], R_VIS);
	}

	public void setPOS(DenseMatrix64F rpos) {
		   CommonOps.mult(1, rpos, R_VIS, R_POS);
	}


	public static String toString(Vector3D_F64 pos) {
			 return String.format("X=% .3f° Y=% .3f° Z=% .3f°",pos.x,pos.y,pos.z);
	}




}
