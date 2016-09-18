/****************************************************************************
 *
 *   Copyright (c) 2016 Eike Mansfeld ecm@gmx.de. All rights reserved.
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

package com.comino.slam.detectors.space;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Map.Entry;

import georegression.struct.point.Point3D_F32;

/**
 * @author ecmnet
 *
 */
public class Space {

	private static final float SPACE_RESOLUTION = 0.5f;

	private static final int SPACE_X_EXTENSION = 5;
	private static final int SPACE_Y_EXTENSION = 10;
	private static final int SPACE_Z_EXTENSION = 5;

	private Map<Integer,SpaceBlock> space = null;
	private Point3D_F32 origin;
	private int total_feature_count=0;

	public Space() {
		this.space = new HashMap<Integer,SpaceBlock>();
		this.origin = new Point3D_F32();

	}

	public void setOrigin(float wx, float wy, float wz) {
		this.origin.x = wx;
		this.origin.y = wy;
		this.origin.z = wz;
	}

	/**
	 * Clears space
	 */
	public void clear() {
		total_feature_count = 0;
		space.clear();
	}

	/**
	 * Add a feature point to the corresponding block of the space
	 * @param point: Feature point
	 */
	public void addFeature(Feature point) {
		SpaceBlock block = getNavigationBlock(point.x,point.y,point.z);
		if(block!=null) {
			block.getFeatures().add(point);
			total_feature_count++;
		}
	}

	public int getTotalFeatureCount() {
		return total_feature_count;
	}

	public Point3D_F32 getMaxFeaturesPositionWorld() {
		int max = 0; Entry<Integer,SpaceBlock> e; int hashBlock=0;

		Iterator<Entry<Integer,SpaceBlock>> i = space.entrySet().iterator();
		while(i.hasNext()) {
			e = i.next();
			if(e.getValue().getFeatures().size()>max) {
				 max = e.getValue().getFeatures().size();
				 hashBlock = e.getKey();
			}
		}

		if(max > 0 && hashBlock>0) {
		    space.get(hashBlock).markAsObstacle(true);

		    // TODO: Mark blocks between origin and found Block as free

		    Point3D_F32 p = calcPoint3DFromHash(hashBlock);
		    p.x = p.x * SPACE_RESOLUTION + origin.x;
		    p.y = p.y * SPACE_RESOLUTION + origin.y;
		    p.z = p.z * SPACE_RESOLUTION + origin.z;
		    return p;
		}
		return null;
	}


	public SpaceBlock getNavigationBlock(float wx, float wy, float wz) {
		int x_index = (int)(( wx  + SPACE_X_EXTENSION / 2f) / SPACE_RESOLUTION - origin.x);
		int y_index = (int)(  wy / SPACE_RESOLUTION - origin.y);
		int z_index = (int)(( wz  + SPACE_X_EXTENSION / 2f) / SPACE_RESOLUTION - origin.z);

		if(x_index > 0 && x_index < SPACE_X_EXTENSION &&
		   y_index > 0 && y_index < SPACE_Y_EXTENSION &&
		   z_index > 0 && z_index < SPACE_Z_EXTENSION)  {

			int blockHash = calcHash(x_index, y_index, z_index);
			SpaceBlock block = space.get(blockHash);
			if(block == null) {
				block = new SpaceBlock();
				space.put(blockHash, block);
			}
			return block;
		}
		return null;
	}

	/* Moves spave to a new origin. If the origin lies in the current space,
	 * the current space is that manner, that the origin is represented by
	 * block(3,0,3)
	 */
	public void moveSpaceToNewOrigin(float wx, float wy, float wz) {

	}

	private int calcHash(int x, int y, int z) {
		return x * 10000 + y * 100 + z;
	}

	private Point3D_F32 calcPoint3DFromHash(int hash) {
		Point3D_F32 p = new Point3D_F32();
		p.x = hash / 10000;
		p.y = (hash % 100) / 100;
		p.z = (hash % 10000);
		return p;
	}

}
