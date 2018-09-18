package com.comino.dev;

import org.ejml.simple.SimpleMatrix;

public class CoVarianceEjML {

    public static void main(String[] args){

        double data[][] = new double[][]{
                { 90, 60, 90 },
                { 90, 90, 30 },
                { 60, 60, 60 },
                { 60, 60, 90 },
                { 30, 30, 30 }
        };

        SimpleMatrix X = new SimpleMatrix(data);
        int n = X.numRows();
        SimpleMatrix Xt = X.transpose();
        int m = Xt.numRows();

        // Means:
        SimpleMatrix x = new SimpleMatrix(m, 1);
        for(int r=0; r<m; r++ ){
            x.set(r, 0, Xt.extractVector(true, r).elementSum() / n);
        }
        // System.out.println(x);

        // Covariance matrix:
        SimpleMatrix S = new SimpleMatrix(m, m);
        for(int r=0; r<m; r++){
            for(int c=0; c<m; c++){
                if(r > c){
                    S.set(r, c, S.get(c, r));
                } else {
                    double cov = Xt.extractVector(true, r).minus( x.get((r), 0) ).dot(Xt.extractVector(true, c).minus( x.get((c), 0) ).transpose());
                    S.set(r, c, (cov / n));
                }
            }
        }
        // System.out.println(S);

        // Plotting:
        for(int r=0; r<m; r++){
            for(int c=0; c<m; c++) { System.out.print(S.get(r, c) + "\t\t\t"); }
            System.out.print("\n");
        }

        // 504.0			360.0			180.0
        // 360.0			360.0			0.0
        // 180.0			0.0  			720.0

    }

}