/*
 * publication.h
 *
 *  Created on: May 10, 2025
 *      Author: jeffr
 */

#ifndef LIB_DATA_BUS_PUBLICATION_H_
#define LIB_DATA_BUS_PUBLICATION_H_

#include "nodes.h"

template<typename T>
class Publisher
{
public:
	Publisher(Node<T>& node) : node(node) {}

	void publish(const T& new_data)
	{
		node.set(new_data);
	}

private:
	Node<T>& node;
};

#endif /* LIB_DATA_BUS_PUBLICATION_H_ */
