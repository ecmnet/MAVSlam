package com.comino.parallel;

import com.comino.parallel.function.Operation;

// http://openimaj.org/tutorial/parallel-processing.html

public class TestParallel {

	public static void main(String[] args) {

		float a;

		System.out.println(Runtime.getRuntime().availableProcessors());

		int delta = 50;

		long t1 = System.currentTimeMillis();
		for(int k=1;k<100;k++)
		for(int i=1;i<delta;i++) {
             op(i);
		}
		System.out.println("Normal: "+(System.currentTimeMillis()-t1)+"ms");

		long t2 = System.currentTimeMillis();
		for(int k=1;k<100;k++)
		Parallel.forIndex(1, delta, 1, new Operation<Integer>() {
			public void perform(Integer i) {
				op(i);
			}
		});
		System.out.println("Parallel: "+(System.currentTimeMillis()-t2)+"ms");

	}

	private static void op(int i) {
		double a;
		for(int j=1;j<10000;j++) { a = (double)i + j +0.1f; }
	}

}
