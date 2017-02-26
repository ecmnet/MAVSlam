package com.comino.slam.vfh.vfh2D;

import java.util.Arrays;

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
}
