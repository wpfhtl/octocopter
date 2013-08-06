#ifndef __BASICGRAPH__
#define __BASICGRAPH__

// INCLUDES
// STL
#include <QVector>
#include <waypoint.h>
#include <utility>
#include <iostream>


struct Path
{
    // the waypoint indices in the order of the shortest path.
	std::vector<unsigned int> vertices;

    unsigned int length;

	// operators
    bool operator==(const struct Path& other) const { return this->length == other.length && this->vertices == other.vertices; }
};


#endif // __BASICGRAPH__
