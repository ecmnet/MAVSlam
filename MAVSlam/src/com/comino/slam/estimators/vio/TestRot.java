package com.comino.slam.estimators.vio;

import com.comino.msp.utils.MSPMathUtils;

import georegression.geometry.ConvertRotation3D_F64;
import georegression.geometry.GeometryMath_F64;
import georegression.struct.EulerType;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;

public class TestRot {

	public static void main(String[] args) {

		Vector3D_F64 offset_body        = new Vector3D_F64();
		Vector3D_F64 offset_ned         = new Vector3D_F64();
		Vector3D_F64 offset_vis         = new Vector3D_F64();
		Vector3D_F64 pos_body           = new Vector3D_F64();

		Se3_F64 current                 = new Se3_F64();
		Se3_F64 current_new             = new Se3_F64();
		Se3_F64 current_new_inv         = new Se3_F64();
		Se3_F64 obs                     = new Se3_F64();


		offset_body.set(-10,0,0);

		current.T.set(5,0,0);

		ConvertRotation3D_F64.eulerToMatrix(EulerType.ZXY,
				0,
				0,
				MSPMathUtils.toRad(0),
				current.R);


		ConvertRotation3D_F64.eulerToMatrix(EulerType.ZXY,
				0,
				0,
				MSPMathUtils.toRad(0),
				obs.R);

		obs.T.set(10,0,0);

		System.out.println(current);

		obs.concat(current, current_new);

		printEuler(current_new);

		current_new.invert(current_new_inv);

		GeometryMath_F64.mult(current_new_inv.R, current_new.T, pos_body);

        pos_body.plusIP(offset_body);

        GeometryMath_F64.mult(current_new.R, pos_body, current_new.T);


		printEuler(current_new);


	}

	private static void printEuler(Se3_F64 m) {
		double[]  visAttitude    = new double[3];
		System.out.print(m);
		ConvertRotation3D_F64.matrixToEuler(m.R, EulerType.ZXY, visAttitude);
		System.out.println(" R:"+MSPMathUtils.fromRad((float)visAttitude[0])+
				" P:"+MSPMathUtils.fromRad((float)visAttitude[1])+
				" Y:"+MSPMathUtils.fromRad((float)visAttitude[2]));
		System.out.println("-"); System.out.println();
	}

}
