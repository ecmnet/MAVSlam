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
public class NavigationSpace {

	private static final float SPACE_RESOLUTION = 0.5f;

	private static final int SPACE_X_EXTENSION = 5;
	private static final int SPACE_Y_EXTENSION = 10;
	private static final int SPACE_Z_EXTENSION = 5;

	private Map<Integer,NavigationBlock> space = null;
	private Point3D_F32 origin;
	private int total_feature_count=0;

	public NavigationSpace() {
		this.space = new HashMap<Integer,NavigationBlock>();
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
	public void addFeature(AttributedPoint3D_F32 point) {
		NavigationBlock block = getNavigationBlock(point.x,point.y,point.z);
		if(block!=null) {
			block.getFeatures().add(point);
			total_feature_count++;
		}
	}

	public int getTotalFeatureCount() {
		return total_feature_count;
	}

	public Point3D_F32 getMaxFeaturesPositionWorld() {
		int max = 0; Entry<Integer,NavigationBlock> e; int hashBlock=0;

		Iterator<Entry<Integer,NavigationBlock>> i = space.entrySet().iterator();
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


	public NavigationBlock getNavigationBlock(float wx, float wy, float wz) {
		int x_index = (int)(( wx  + SPACE_X_EXTENSION / 2f) / SPACE_RESOLUTION - origin.x);
		int y_index = (int)(  wy / SPACE_RESOLUTION - origin.y);
		int z_index = (int)(( wz  + SPACE_X_EXTENSION / 2f) / SPACE_RESOLUTION - origin.z);

		if(x_index > 0 && x_index < SPACE_X_EXTENSION &&
		   y_index > 0 && y_index < SPACE_Y_EXTENSION &&
		   z_index > 0 && z_index < SPACE_Z_EXTENSION)  {

			int blockHash = calcHash(x_index, y_index, z_index);
			NavigationBlock block = space.get(blockHash);
			if(block == null) {
				block = new NavigationBlock();
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
