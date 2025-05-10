/*
 * subscription.h
 *
 *  Created on: May 10, 2025
 *      Author: jeffr
 */

#ifndef LIB_DATA_BUS_SUBSCRIPTION_H_
#define LIB_DATA_BUS_SUBSCRIPTION_H_

#include "nodes.h"

template<typename T>
class Subscriber
{
public:
	Subscriber(Node<T>& node) : node(node) {}

	bool check_new()
	{
		return node.check_new(last_timestamp);
	}

	T get()
	{
		return node.get(&last_timestamp);
	}

private:
	Node<T>& node;
	uint64_t last_timestamp = 0;
};

#endif /* LIB_DATA_BUS_SUBSCRIPTION_H_ */
