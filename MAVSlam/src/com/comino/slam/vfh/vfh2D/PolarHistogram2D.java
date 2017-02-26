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

import com.comino.slam.vfh.VfhGrid;
import com.comino.slam.vfh.VfhHist;

public class PolarHistogram2D {

	private VfhHist hist;

	public PolarHistogram2D(int alpha, double threshold, double density_a, double density_b) {

	  hist = new VfhHist();
	  hist.alpha = alpha;
	  hist.sectors = 360 / alpha;
	  hist.threshold = threshold;
	  hist.densities = new short[hist.sectors];

	  Arrays.fill(hist.densities, (short)0);

	}

	public void histUpdate(VfhGrid grid) {
	   int dim = grid.dimension;

	   for (int i = 0; i < dim; ++i) {
		    for (int j = 0; j < dim; ++j) {

		    	 /* Calculate the angular position (beta) of this cell. */
		    	  double beta = Math.atan2((double)(j - dim/2), (double)(i - dim/2));

		    	  /* Calculate the obstacle density of this cell. */
			      double density = Math.pow(grid.cells[i * dim + j], 2);
			      density *= hist.density_a - hist.density_b * Math.sqrt((i - dim/2)*(i - dim/2) + (j - dim/2)*(j - dim/2));

			      /* Add density to respective point in the histogram. */
			      hist.densities[(int) Math.floor(beta / hist.alpha)] += density;
		    }
	   }
	}
}
