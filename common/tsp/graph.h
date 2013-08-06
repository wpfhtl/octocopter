#ifndef GRAPH_H
#define GRAPH_H

#include "path.h"

struct AdjacencyMatrix
{
    // a vector of coordinates
    QVector<WayPoint> entries;
    unsigned int size;

    // operators for getting costs
    const unsigned int operator()(unsigned int row, unsigned int col) const
    {
        return
                abs(entries[row].x() - entries[col].x()) +
                abs(entries[row].y() - entries[col].y()) +
                abs(entries[row].z() - entries[col].z());
    }

    /*unsigned int operator()(unsigned int row, unsigned int col)
    {
        return abs(entries[row].first - entries[col].first) + abs(entries[row].second - entries[col].second);
    }*/

    // constructors
    AdjacencyMatrix(QVector<WayPoint> e) : entries(e), size(entries.size()) {}

    AdjacencyMatrix() : size(0) {}
};

class Graph
{
	unsigned int numVertices;
    AdjacencyMatrix adjMat;

	// helpers for traveling salesperson
	struct Path naiveTSPHelper(struct Path soFar, std::vector<bool> visited);

public:
    void setWayPointList(const QList<WayPoint>* const wpl);

	// getters and setters
	const unsigned int operator()(unsigned int row, unsigned int col) const
    { return adjMat(row, col); }

	unsigned int operator()(unsigned int row, unsigned int col)
    { return adjMat(row, col); }

    unsigned int getNumVertices() { return numVertices; }

	// TSP Methods
	struct Path naiveTSP(); // for testing
	struct Path fastTSP(bool useOpt); 
	struct Path optTSP();
	struct Path optTSP(struct Path& fast);

	// validation methods
	bool isValidPath(const struct Path& p) const;

    friend std::ostream& operator<<(std::ostream& os, const Graph& graph);
};

#endif // __GRAPH__
