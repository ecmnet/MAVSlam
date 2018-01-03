package com.comino.dev;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import com.comino.msp.utils.MSPMathUtils;

import georegression.geometry.ConvertRotation3D_F64;
import georegression.struct.EulerType;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;

public class RotationMatrixTest {

	private Vector3D_F64 pos         = new Vector3D_F64();
	private Vector3D_F64 ori         = new Vector3D_F64();
	private Vector3D_F64 pos_rotated = new Vector3D_F64();
	private Vector3D_F64 pos_2       = new Vector3D_F64();

	private DenseMatrix64F  R  = new DenseMatrix64F(3,3);
	private DenseMatrix64F  RT  = new DenseMatrix64F(3,3);
	private DenseMatrix64F  RE  = new DenseMatrix64F(3,3);

	private Se3_F64 trans   = new Se3_F64();
	private Se3_F64 invert  = new Se3_F64();
	private Se3_F64 trans2  = new Se3_F64();
	private Se3_F64 point   = new Se3_F64();
	private Se3_F64 result  = new Se3_F64();
	private Se3_F64 point2  = new Se3_F64();


	private double[]  visAttitude    = new double[3];



	private DenseMatrix64F  P  = DenseMatrix64F.wrap(3,3, new double[]
			{ 1d , 0d , 0d ,
			  0d , 1d , 0d ,
			  0d , 0d,  1d }
			);

	public RotationMatrixTest() {

		point.getTranslation().set(1,1,0);
		ConvertRotation3D_F64.eulerToMatrix(EulerType.ZXY,
				MSPMathUtils.toRad(45),
			    MSPMathUtils.toRad(0),
				MSPMathUtils.toRad(0),point.getRotation());

		System.out.println(point);

		trans.getTranslation().set(0,0,0);
		ConvertRotation3D_F64.eulerToMatrix(EulerType.ZXY,
				MSPMathUtils.toRad(45),
			    MSPMathUtils.toRad(0),
				MSPMathUtils.toRad(0),trans.getRotation());

       CommonOps.invert(point.R,invert.R);
       point.concat(invert, point2);


   //    point.concat(point2, trans2);



      point2.concat(trans, result);



		System.out.println(result);

//		trans2.getTranslation().set(0,0,0);
//		ConvertRotation3D_F64.eulerToMatrix(EulerType.ZXY,
//				MSPMathUtils.toRad(45),
//			    MSPMathUtils.toRad(0),
//				MSPMathUtils.toRad(0),trans2.getRotation());

//		point.concat(trans, point2);
//		point2.concat(trans2, result);


	  System.out.println(MSPMathUtils.fromRad((float)Math.acos(result.T.x / (Math.sqrt(result.T.x*result.T.x+result.T.y*result.T.y)))));


//		System.out.println(result.T);
//
//        P.set(0, 0, result.getX());
//        P.set(0, 1, result.getY());
//        P.set(0, 2, result.getZ());
//
//		ConvertRotation3D_F64.matrixToEuler(P, EulerType.ZXY, visAttitude);
//
//		System.out.println(" R:"+MSPMathUtils.fromRad((float)visAttitude[0])+
//				           " P:"+MSPMathUtils.fromRad((float)visAttitude[1])+
//				           " Y:"+MSPMathUtils.fromRad((float)visAttitude[2]));
//


//		pos.x = 2; pos.y = 1; pos.z = 0;
//		System.out.println(RotationModel.toString(pos));
//		System.out.println();
//
//
//
//
//		ConvertRotation3D_F64.eulerToMatrix(EulerType.ZXY,
//				MSPMathUtils.toRad(0),
//			    MSPMathUtils.toRad(0),
//				MSPMathUtils.toRad(-90),R);
//
//		model.setVIS( new double[] { MSPMathUtils.toRad(0), MSPMathUtils.toRad(0), MSPMathUtils.toRad(90) });
//        model.setPOS(R);
//
//		GeometryMath_F64.mult(model.R_VIS, pos, pos_rotated);
//
//		System.out.println(RotationModel.toString(pos_rotated));
//
//		GeometryMath_F64.mult(model.R_POS, pos_rotated, pos_rotated);
//
//		System.out.println(RotationModel.toString(pos_rotated));

//		float[] angle = new float[3];
//		ConvertRotation3D_F32.matrixToEuler(model.R_VIS, EulerType.YZX,angle);
//
//		System.out.println();
//		System.out.println(RotationModel.toString(angle));


	}

	public static void main(String[] args) {
		new RotationMatrixTest();
	}

}
