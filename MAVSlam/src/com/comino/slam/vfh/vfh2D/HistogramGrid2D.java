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

import java.util.Arrays;

import com.comino.msp.model.segment.Slam;
import com.comino.slam.vfh.VfhGrid;

import georegression.struct.point.Point3D_F64;

public class HistogramGrid2D {

	private VfhGrid grid = null;

	public HistogramGrid2D(float dimension, float resolution) {
		assert(dimension % 2 == 1);

		grid = new VfhGrid();
		grid.dimension  = (int)Math.floor(dimension / resolution + 1);
		grid.resolution = resolution;
		grid.cells = new short[grid.dimension * grid.dimension];

		Arrays.fill(grid.cells,(short)0);
	}

	// Updates the grid with an absolute observation
	public boolean gridUpdate(Point3D_F64 obstacle_absolute) {
		return gridUpdate(0,0,obstacle_absolute);
	}

	// Updates the grid with an relative observation
	public boolean gridUpdate(float lpos_x, float lpos_y, Point3D_F64 obstacle) {

		int new_x = (int)Math.floor(lpos_x / grid.resolution) + (int)Math.floor(obstacle.x / grid.resolution);
		int new_y = (int)Math.floor(lpos_y / grid.resolution) + (int)Math.floor(obstacle.y / grid.resolution);

		if (new_x < grid.dimension && new_y < grid.dimension) {
			grid.cells[new_x * grid.dimension + new_y] += (short)1;
			return true;
		}
		return false;
	}

	// creates a moveing window at the current position with a certain window size
	public VfhGrid getMovingWindow(float lpos_x, float lpos_y, int windowSize) {
		VfhGrid window = new VfhGrid();

		for (int i = 0; i < windowSize; ++i) {
			for (int j = 0; j < windowSize; ++j) {
				int grid_i = i + (int)Math.floor(lpos_x / grid.resolution) + (windowSize - 1) / 2;
				int grid_j = j + (int)Math.floor(lpos_y / grid.resolution) + (windowSize - 1) / 2;

				if (grid_i < grid.dimension && grid_j < grid.dimension) {
					window.cells[i * windowSize + j] = grid.cells[grid_i * grid.dimension + grid_j];
				}
			}
		}
		return window;
	}

	public void transferToMicroSLAM(Slam slam, int threshold, boolean debug) {
		slam.clear();
		for (int i = 0; i < grid.dimension; ++i) {
			for (int j = 0; j < grid.dimension; ++j) {
				if(grid.cells[i * grid.dimension + j] > threshold)
					slam.setBlock(j*grid.resolution/100f+grid.resolution/200f,
							i*grid.resolution/100f+grid.resolution/200f);
			}
		}
		if(debug)
			System.out.println(slam);
	}
}
