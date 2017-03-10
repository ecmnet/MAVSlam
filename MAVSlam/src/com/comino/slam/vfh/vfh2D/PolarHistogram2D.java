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

import com.comino.msp.utils.MSPMathUtils;
import com.comino.slam.vfh.VfhGrid;
import com.comino.slam.vfh.VfhHist;

public class PolarHistogram2D {

	private VfhHist hist;
	private VfhHist hist_smoothed;

	private float resolution;
	private float density_a;
	private float density_b;

	private int threshold;

	public PolarHistogram2D(int alpha, int threshold, float density_a, float density_b, float resolution) {

		hist          = new VfhHist(alpha);
		hist_smoothed = new VfhHist(alpha);

		this.density_a = density_a;
		this.density_b = density_b;
		this.threshold = threshold;

		this.resolution = resolution;

		Arrays.fill(hist.densities, (int)0);

	}

	public void histUpdate(VfhGrid grid) {
		int dim = grid.dimension; double density=0;

		Arrays.fill(hist.densities, (int)0);
		for (int i = 0; i < dim; ++i) {
			for (int j = 0; j < dim; ++j) {

				if(grid.cells[i * dim + j] < threshold)
					continue;

				/* Calculate the angular position (beta) of this cell. */
				int beta = (int)MSPMathUtils.fromRad((float)Math.atan2((double)(i - dim/2), (double)(j - dim/2)));

				/* Calculate the obstacle density of this cell. */
				density = grid.cells[i * dim + j] * grid.cells[i * dim + j] *
						(density_a - density_b * Math.sqrt((i - dim/2)*(i - dim/2) + (j - dim/2)*(j - dim/2)) * resolution);

				/* Add density to respective point in the histogram. */
				if(beta > -180 && beta < 180)
					hist.densities[(int)(beta+180) / hist.alpha] += (int)density;
			}
		}
	}

	public VfhHist histSmooth(int l) {
		int h;
		for(int k =0; k < hist.sectors; k++) {
			h = 0;
			for(int i = -l; i<= l; i++ )
				h  = h + hist.densities[wrap(i+l,hist.sectors)] * (l - Math.abs(i) + 1);
			hist_smoothed.densities[k] = h / (2 * l + 1);
		}
		return hist_smoothed;
	}



	private int wrap(int s, int max) {
		if(s < 0)
			return s + max;
		if(s > max)
			return s - max;
		return s;
	}


	public String toString() {
		StringBuilder b = new StringBuilder();
		for(int i=0;i<hist.sectors;i++) {
			if(hist.densities[i]<threshold)
				b.append(".");
			else
				b.append("X");
		}
		return b.toString();
	}

}
