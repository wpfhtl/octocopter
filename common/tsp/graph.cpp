#include "graph.h"
#include <waypoint.h>

void Graph::setWayPointList(const QList<WayPoint>* const wpl)
{
    QVector<WayPoint> wayPoints;

    // completely useless, i believe
    numVertices = wpl->size();
    wayPoints.reserve(numVertices);

    for(int i=0;i<wpl->size();i++)
        wayPoints.push_back(wpl->at(i));

    // build the adjacency "matrix" (really just a vector)
    this->adjMat = AdjacencyMatrix(wayPoints);
}


// for testing
bool Graph::isValidPath(const struct Path& p) const
{
	// calculate the distance from each edge, make sure sum is correct
	unsigned int length = 0;
	for(unsigned int i = 0; i < p.vertices.size(); ++i)
	{
        length += this->operator()(p.vertices[i], p.vertices[(i + 1) % p.vertices.size()]);
	}

	return length == p.length;
}
