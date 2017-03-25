/****************************************************************************
 *
 *   Copyright (c) 2017 Eike Mansfeld ecm@gmx.de. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * Source: https://github.com/agarie/vector-field-histogram
 */

package com.comino.slam.vfh.vfh2D;

import com.comino.msp.main.StartUp;
import com.comino.msp.model.DataModel;
import com.comino.msp.model.segment.Grid;
import com.comino.msp.model.segment.Slam;
import com.comino.slam.vfh.VfhGrid;
import com.comino.slam.vfh.VfhHist;

import georegression.struct.point.Point3D_F64;

public class HistogramGrid2D {

	private static final long OBLIVISION_TIME_MS = 10000;
	private static final int  MAX_CERTAINITY     = 1000;

	private VfhGrid grid     = null;
	private VfhGrid window   = null;

	private long        tms  = 0;
	private float   centerx  = 0;
	private float   centery  = 0;

	public HistogramGrid2D(float cenx, float ceny, float dimension,  float windowsize, float resolution) {
		assert(dimension % 2 == 1);

		centerx = cenx;
		centery = ceny;
		grid   = new VfhGrid(dimension, resolution);
		window = new VfhGrid(windowsize, resolution);
	}

	public void reset(DataModel model) {
		grid.clear();
		for (int i = 0; i < grid.dimension; ++i) {
			for (int j = 0; j < grid.dimension; ++j) {
				model.grid.setBlock(j*grid.resolution/100f-centerx,i*grid.resolution/100f-centery, false);
			}
		}
	}

	// Updates the grid with an absolute observation
	public boolean gridUpdate(Point3D_F64 obstacle_absolute) {
		return gridUpdate(0,0,obstacle_absolute);
	}

	// Updates the grid with an relative observation
	public boolean gridUpdate(float lpos_x, float lpos_y, Point3D_F64 obstacle) {

		int new_x = (int)Math.floor((lpos_x+centerx+obstacle.x)*100f / grid.resolution);
		int new_y = (int)Math.floor((lpos_y+centery+obstacle.y)*100f / grid.resolution);

		if (new_x < grid.dimension && new_y < grid.dimension && new_x > 0 && new_y > 0
				&& grid.cells[new_y * grid.dimension + new_x]<MAX_CERTAINITY) {
			grid.cells[new_y * grid.dimension + new_x] += 1;
			return true;
		}
		return false;
	}

	// creates a moveing window at the current position with a certain window size
	public synchronized VfhGrid getMovingWindow(float lpos_x, float lpos_y) {
		window.clear(); int new_x  = 0; int new_y = 0;
		for (int y = 0; y < window.dimension; y++) {
			for (int x = 0; x < window.dimension;x++) {
				new_x = x + (int)Math.floor((lpos_x+centerx)*100f / grid.resolution) - (window.dimension - 1) / 2;
				new_y = y + (int)Math.floor((lpos_y+centery)*100f / grid.resolution) - (window.dimension - 1) / 2;

				if (new_x < grid.dimension && new_y < grid.dimension && new_x >= 0 && new_y >= 0) {
					window.cells[y * window.dimension + x] = grid.cells[new_y * grid.dimension + new_x];
				}
			}
		}
		return window;
	}

	public void transferGridToModel(DataModel model, int threshold, boolean debug) {
		for (int i = 0; i < grid.dimension; ++i) {
			for (int j = 0; j < grid.dimension; ++j) {

				if(grid.cells[i * grid.dimension + j] == 0)
					continue;

				if(grid.cells[i * grid.dimension + j] > threshold) {
					model.grid.setBlock(j*grid.resolution/100f-centerx,i*grid.resolution/100f-centery, true);
					//	System.out.println("ADD: "+(j*grid.resolution/100f-center_x)+ ":"+ (i*grid.resolution/100f-center_y));
				}
				else
					model.grid.setBlock(j*grid.resolution/100f-centerx,i*grid.resolution/100f-centery, false);
			}
		}
		if(debug)
			System.out.println(model.grid);
	}

	public String toString(int threshold) {
		return grid.toString(threshold);
	}


	public void forget() {
		if((System.currentTimeMillis()-tms)>OBLIVISION_TIME_MS) {
			tms = System.currentTimeMillis();
			for (int i = 0; i < grid.dimension; ++i)
				for (int j = 0; j < grid.dimension; ++j)
					if(grid.cells[i * grid.dimension + j] > 1)
						grid.cells[i * grid.dimension + j] -= 1;
		}
	}

	public static void main(String[] args) {
		Grid grid = new Grid(5,0.1f);
		grid.setIndicator(0.1f, 0.1f);

		HistogramGrid2D hist = new HistogramGrid2D(5,5,10,1f,0.1f);

		Point3D_F64 p = new Point3D_F64();

		for(int c= 0; c < 5; c++) {

			for(int i=0;i<10;i++) {
				p.set(0.5f,i/10f-0.15f,0); hist.gridUpdate(p);
			}

			for(int i=0;i<10;i++) {
				p.set(i/10f-1.15f,0.5f,0); hist.gridUpdate(p);
			}
		}

		PolarHistogram2D  pol = new PolarHistogram2D(3,2,3,0.1f,0.1f);
		VfhGrid window = hist.getMovingWindow(0.1f,0.1f);
		System.out.println(window.toString(0));
		pol.histUpdate(window);


		pol.print();
		VfhHist s = pol.histSmooth(5);
		int vi = pol.selectValley(s,240);
		int d = pol.getDirection(s,vi,12);
	    System.out.println(d);
		pol.print(pol.histSmooth(5),d);



	}

}
