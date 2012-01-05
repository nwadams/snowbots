/*
 * VectorPath.h
 *
 *  Created on: 2010-12-28
 *      Author: nwadams
 */

#include <iostream>
#include <vector>
#include "ConePosition.h"

#ifndef VECTORPATH_H_
#define VECTORPATH_H_

namespace std {

class VectorPath {
public:
	VectorPath(vector<Cone> leftwall, vector<Cone> rightwall);
	virtual ~VectorPath();
	vector<Cone> path;

	void printPath();
	void printGraph();

private:
	void addToPath(Coordinate point);
};

}

#endif /* VECTORPATH_H_ */
