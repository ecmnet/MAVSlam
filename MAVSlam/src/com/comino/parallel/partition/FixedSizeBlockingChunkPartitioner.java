package com.comino.parallel.partition;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Queue;

/**
 * A {@link FixedSizeBlockingChunkPartitioner} dynamically partitions data into
 * chunks of a fixed length. The partitioner does not need to know about the
 * length of the data apriori. Beyond a {@link FixedSizeChunkPartitioner} this
 * implementation doesn't use an underlying iterator. It can only be
 * instantiated on queues and will always report to have more items available
 * while it waits for the queue to be filled.
 *
 * @author Jonathon Hare (jsh2@ecs.soton.ac.uk)
 *
 * @param <T>
 *            Type of object in the partition
 */
public class FixedSizeBlockingChunkPartitioner<T> implements Partitioner<T> {
	private Queue<T> queue;
	private int chunkSize = 20;

	/**
	 * Construct with data in the form of a {@link Queue}. The number of items
	 * per chunk is set at the default value (20).
	 *
	 * @param objects
	 *            the {@link Queue} representing the data.
	 */
	public FixedSizeBlockingChunkPartitioner(Queue<T> objects) {
		this.queue = objects;
	}

	/**
	 * Construct with data in the form of an {@link Queue} and the given number
	 * of items per chunk.
	 *
	 * @param objects
	 *            the {@link Queue} representing the data.
	 * @param chunkSize
	 *            number of items in each chunk.
	 */
	public FixedSizeBlockingChunkPartitioner(Queue<T> objects, int chunkSize) {
		this.queue = objects;
		this.chunkSize = chunkSize;
	}

	@Override
	public Iterator<Iterator<T>> getPartitions() {
		return new Iterator<Iterator<T>>() {

			@Override
			public boolean hasNext() {
				return true;
			}

			@Override
			public Iterator<T> next() {

				final List<T> list = new ArrayList<T>(chunkSize);

				int i = 0;
				while (i < chunkSize) {
					T toAdd = null;
					synchronized (queue) {
						toAdd = queue.poll();
					}
					if (toAdd == null) {
						continue;
					}
					list.add(toAdd);
					i++;
				}

				return list.iterator();

			}

			@Override
			public void remove() {
				throw new UnsupportedOperationException("Not supported");
			}
		};
	}
}